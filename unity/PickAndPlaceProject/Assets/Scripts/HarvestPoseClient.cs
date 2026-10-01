using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Ur10eRg2Moveit;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

/// Runs the pick-and-place sequence: ask MoveIt for each pose, play the trajectory, open/close the gripper.
/// Arm motion lives in ArmController, gripper in GripperController.
public class HarvestPoseClient : MonoBehaviour
{
    const string ServiceName = "harvest_pose";

    [Header("Components")]
    public ArmController arm;
    public GripperController gripper;
    [Tooltip("Optional. If set, obstacles are re-published to MoveIt on every click.")]
    public PlanningSceneSync sceneSync;

    [Header("Scene references")]
    public Transform tomatoTarget;      // the cube for now, the tomato later
    public Transform placementTarget;
    [Tooltip("Empty child of tool0 between the fingertips. +Z = toward fingertips, +Y = finger closing axis.")]
    public Transform gripAttachPoint;

    [Header("Tool frame")]
    public float graspYawOffset = 0f;
    public Vector3 eeFrameCorrectionEuler = new Vector3(-90f, 0f, 0f);

    [Header("Motion")]
    public float approachHeight = 0.05f;
    public float verticalStepSize = 0.2f;
    public float placeClearance = 0.005f;
    public float surfaceRayHeight = 1f;
    [Tooltip("How far the finger pads reach below the grip attach point (m).")]
    public float fingertipDrop = 0.01f;
    public float tableClearance = 0.01f;
    public float serviceTimeout = 15f;

    [Header("Grasp")]
    [Range(0f, 1f)] public float graspHeightFraction = 1.0f;
    public float objectMass = 0.3f;
    public float objectFriction = 1.5f;
    public float gripHoldTime = 0.4f;
    [Tooltip("Max distance between object centre and tool point after closing (m).")]
    public float maxGraspGap = 0.03f;
    [Tooltip("Abort if the object drifts this far from the tool after closing (m).")]
    public float maxHoldDrift = 0.04f;

    [Header("Grasp assist (keeps the object attached while the fingers hold it)")]
    public bool useGraspAssist = true;
    public float assistBreakForce = 500f;
    [Tooltip("Optional. Joint holding the fruit to the plant (peduncle). Destroyed right after a successful grasp.")]
    public Joint stemJoint;

    [Header("Debug")]
    public bool verbose = true;

    ROSConnection ros;
    bool running, serverReset, stepOk;
    float lastGripperCmd = float.NaN;
    FixedJoint assistJoint;

    enum Kind { Move, Grasp, Release }
    struct Step
    {
        public Kind kind; public Vector3 pos; public float grip; public string label;
        public Step(Kind k, Vector3 p, float g, string l) { kind = k; pos = p; grip = g; label = l; }
    }

    void Log(string tag, string msg) { if (verbose) Debug.Log($"[{tag}] {msg}"); }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<HarvestPoseRequest, HarvestPoseResponse>(ServiceName);
    }

    // ----------------------------------------------------------------- entry

    public void OnHarvestButtonClicked()
    {
        if (running) { Debug.LogWarning("Harvest already running."); return; }
        if (sceneSync != null) sceneSync.PublishAll();
        StartCoroutine(Run());
    }

    IEnumerator Run()
    {
        running = true;
        bool ok = false;
        if (Validate())
        {
            if (!serverReset) yield return ResetServerState();
            lastGripperCmd = float.NaN;
            var steps = BuildSteps(out Quaternion grasp);
            if (steps != null) { yield return Execute(steps, grasp, r => ok = r); }
        }
        DestroyAssist();
        Debug.Log(ok ? "Harvest sequence complete." : "Harvest sequence ABORTED.");
        running = false;
    }

    bool Validate()
    {
        var missing = new List<string>();
        if (arm == null) missing.Add("HarvestPoseClient.arm");
        else if (arm.robotBase == null) missing.Add("ArmController.robotBase");
        if (gripper == null) missing.Add("HarvestPoseClient.gripper");
        else if (gripper.arm == null) missing.Add("GripperController.arm");
        if (tomatoTarget == null) missing.Add("tomatoTarget");
        if (placementTarget == null) missing.Add("placementTarget");
        if (gripAttachPoint == null) missing.Add("gripAttachPoint");

        if (missing.Count > 0)
        {
            Debug.LogError("[Harvest] Unassigned in Inspector: " + string.Join(", ", missing));
            return false;
        }
        return true;
    }

    // Auto-fill arm/gripper when the components sit on the same GameObject.
    void Reset()
    {
        arm = GetComponent<ArmController>();
        gripper = GetComponent<GripperController>();
    }

    void Awake()
    {
        if (arm == null) arm = GetComponent<ArmController>();
        if (gripper == null) gripper = GetComponent<GripperController>();
        if (gripper != null && gripper.arm == null) gripper.arm = arm;
    }

    // ----------------------------------------------------------- step building

    List<Step> BuildSteps(out Quaternion grasp)
    {
        grasp = ComputeGraspRotation(tomatoTarget);

        // Pick point: a fraction up the object, but never so low that the fingertips hit the table.
        Bounds tb = WorldBounds(tomatoTarget);
        Vector3 pickPoint = tb.center;
        pickPoint.y = tb.min.y + tb.size.y * graspHeightFraction;
        if (TryFindSurface(pickPoint, out Vector3 table))
        {
            float minY = table.y + fingertipDrop + tableClearance;
            if (minY > pickPoint.y) Log("Reach", $"tool y clamped {pickPoint.y:F3} -> {minY:F3} (table y={table.y:F3})");
            pickPoint.y = Mathf.Max(pickPoint.y, minY);
            if (pickPoint.y > tb.max.y - 0.01f) Debug.LogWarning("[Reach] Object too low to grasp above the table.");
        }
        else Debug.LogWarning("[Reach] No surface under object; fingertip clamp not applied.");

        // Place point: object centre above the placement surface.
        Vector3 placePoint = placementTarget.position;
        if (TryFindSurface(placePoint, out Vector3 surface)) placePoint = surface;
        placePoint += Vector3.up * (tb.extents.y + placeClearance);

        PrepareObject(tomatoTarget);

        if (Vector3.Dot(gripAttachPoint.localPosition.normalized, gripAttachPoint.localRotation * Vector3.forward) < 0.99f)
            Debug.LogWarning("gripAttachPoint +Z does not point from the flange toward the fingertips.");

        Vector3 pick = Flange(pickPoint, grasp), place = Flange(placePoint, grasp), up = Vector3.up * approachHeight;
        float O = gripper.open, C = gripper.closed;

        var s = new List<Step> { new Step(Kind.Move, pick + up, O, "Hover") };
        AddLinear(s, pick + up, pick, O, "Descend");
        s.Add(new Step(Kind.Grasp, pick, C, "Grasp"));
        AddLinear(s, pick, pick + up, C, "Lift");
        s.Add(new Step(Kind.Move, place + up, C, "Move above basket"));
        AddLinear(s, place + up, place, C, "Lower");
        s.Add(new Step(Kind.Release, place, O, "Release"));
        AddLinear(s, place, place + up, O, "Retreat");
        return s;
    }

    void AddLinear(List<Step> steps, Vector3 from, Vector3 to, float grip, string label)
    {
        int n = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(from, to) / verticalStepSize));
        for (int i = 1; i <= n; i++)
            steps.Add(new Step(Kind.Move, Vector3.Lerp(from, to, i / (float)n), grip, $"{label} {i}/{n}"));
    }

    // --------------------------------------------------------------- execution

    IEnumerator Execute(List<Step> steps, Quaternion grasp, System.Action<bool> done)
    {
        float baseline = 0f;
        bool holding = false;

        foreach (var s in steps)
        {
            if (s.kind == Kind.Release) DestroyAssist(); // must be gone before the fingers open

            // Grasp / Release do not move the arm, so only Move steps are planned.
            if (s.kind == Kind.Move)
            {
                yield return PlanAndPlay(s, grasp);
                if (!stepOk) { done(false); yield break; }

                if (s.label.StartsWith("Descend") && FlangeError(s.pos) > 0.04f)
                {
                    Debug.LogError($"[{s.label}] Descent blocked, flange {FlangeError(s.pos):F3} m from target. Not closing.");
                    LogBlockers();
                    done(false); yield break;
                }
            }

            // Gripper: settle the arm first so the object is not flung.
            if (float.IsNaN(lastGripperCmd) || !Mathf.Approximately(lastGripperCmd, s.grip))
            {
                yield return arm.WaitSettled();
                gripper.Set(s.grip);
                lastGripperCmd = s.grip;
                yield return gripper.WaitSettled();
            }

            if (s.kind == Kind.Grasp)
            {
                yield return new WaitForSeconds(gripHoldTime);
                float gap = ObjectToTool();
                Log("Grasp", $"lead={gripper.LeadPosition:F2} rad, holdsObject={gripper.HoldsObject()}, object-to-tool={gap:F3} m");

                if (!gripper.HoldsObject()) { Debug.LogError("Grasp failed: fingers closed fully (nothing between them)."); done(false); yield break; }
                if (gap > maxGraspGap)      { Debug.LogError($"Grasp failed: tool is {gap:F3} m from the object."); done(false); yield break; }

                baseline = gap;
                holding = true;
                if (stemJoint != null) { Destroy(stemJoint); stemJoint = null; Log("Grasp", "Stem joint released."); }
                if (useGraspAssist) CreateAssist();
            }
            else if (s.kind == Kind.Release)
            {
                holding = false;
            }
            else if (holding)
		{
		    yield return arm.WaitSettled();   // measure after the arm stops, not mid-motion
		    float drift = ObjectToTool() - baseline;
		    Log("Hold", $"{s.label}: drift={drift:F3} m, assistJoint alive={assistJoint != null}");
		    if (drift > maxHoldDrift) { Debug.LogError($"Object slipped (drift {drift:F3} m)."); done(false); yield break; }
		}
        }
        done(true);
    }

    IEnumerator PlanAndPlay(Step s, Quaternion grasp)
    {
        stepOk = false;
        Transform rb = arm.robotBase;

        Vector3 localPos = rb.InverseTransformPoint(s.pos);
        Quaternion sent = grasp * Quaternion.Euler(eeFrameCorrectionEuler);
        Quaternion localRot = Quaternion.Inverse(rb.rotation) * sent;

        var req = new HarvestPoseRequest();
        req.target_pose = new PoseMsg(
            new PointMsg(localPos.z, -localPos.x, localPos.y),                        // Unity -> ROS axes
            new QuaternionMsg(-localRot.z, localRot.x, -localRot.y, localRot.w));
        req.gripper_position = Mathf.Clamp01(Mathf.InverseLerp(gripper.open, gripper.closed, s.grip));

        HarvestPoseResponse res = null;
        bool got = false;
        Log(s.label, $"request ROS pos=({localPos.z:F3}, {-localPos.x:F3}, {localPos.y:F3})");
        ros.SendServiceMessage<HarvestPoseResponse>(ServiceName, req, r => { res = r; got = true; });

        float t0 = Time.realtimeSinceStartup;
        yield return new WaitUntil(() => got || Time.realtimeSinceStartup - t0 > serviceTimeout);

        if (!got) { Debug.LogError($"[{s.label}] Service timed out. Is the ROS side running?"); yield break; }
        if (res == null || !res.success) { Debug.LogError($"[{s.label}] Planning failed: {res?.message}"); yield break; }
        if (res.trajectory == null || res.trajectory.points.Length == 0) { Debug.LogError($"[{s.label}] Empty trajectory."); yield break; }

        Log(s.label, $"plan OK, {res.trajectory.points.Length} points");
        yield return arm.Play(res.trajectory);
        stepOk = true;
    }

    /// Tells the MoveIt server to forget its tracked joint state (gripper_position < 0). Once per Play session.
    IEnumerator ResetServerState()
    {
        bool got = false;
        var req = new HarvestPoseRequest
        {
            target_pose = new PoseMsg(new PointMsg(0, 0, 0), new QuaternionMsg(0, 0, 0, 1)),
            gripper_position = -1f
        };
        ros.SendServiceMessage<HarvestPoseResponse>(ServiceName, req, r => got = true);
        float t0 = Time.realtimeSinceStartup;
        yield return new WaitUntil(() => got || Time.realtimeSinceStartup - t0 > serviceTimeout);
        serverReset = got;
        Log("Reset", got ? "server state reset" : "no reply, continuing");
    }

    // ------------------------------------------------------------- grasp physics

    /// Rigidbody + friction so the object behaves when squeezed.
    /// Rigidbody + friction so the object behaves when squeezed.
    void PrepareObject(Transform obj)
    {
        var rb = obj.GetComponent<Rigidbody>() ?? obj.gameObject.AddComponent<Rigidbody>();
        rb.isKinematic = false;
        rb.mass = objectMass;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.solverIterations = 30;
        rb.solverVelocityIterations = 10;
        rb.maxDepenetrationVelocity = 0.3f;

        var mat = new PhysicMaterial("GripMat")
        {
            dynamicFriction = objectFriction, staticFriction = objectFriction, bounciness = 0f,
            frictionCombine = PhysicMaterialCombine.Maximum, bounceCombine = PhysicMaterialCombine.Minimum
        };
        
        // Grab all colliders on the target object
        var targetColliders = obj.GetComponentsInChildren<Collider>();
        foreach (var c in targetColliders) c.sharedMaterial = mat;

        // Iterate through all colliders on the robot arm/gripper
        foreach (var gripperCol in gripAttachPoint.root.GetComponentsInChildren<Collider>())
        {
            if (gripperCol.name.ToLower().Contains("finger")) 
            {
                // Apply the friction material to the fingers
                gripperCol.sharedMaterial = mat;
            }
            else 
            {
                // Ignore collisions between the knuckles/base and the target object
                foreach (var targetCol in targetColliders)
                {
                    Physics.IgnoreCollision(gripperCol, targetCol, true);
                }
            }
        }

        foreach (var ab in arm.Root.GetComponentsInChildren<ArticulationBody>())
            if (ab.isRoot) { ab.solverIterations = 64; ab.solverVelocityIterations = 16; }
    }

    /// Fixed joint from the object to the tool: guarantees it travels with the gripper once the fingers have closed on it.
    void CreateAssist()
    {
        var rb = tomatoTarget.GetComponent<Rigidbody>();
        var tool = gripAttachPoint.GetComponentInParent<ArticulationBody>();
        if (rb == null || tool == null) { Debug.LogWarning("[Assist] Missing Rigidbody or tool ArticulationBody."); return; }
        assistJoint = tomatoTarget.gameObject.AddComponent<FixedJoint>();
        assistJoint.connectedArticulationBody = tool;
        assistJoint.breakForce = assistBreakForce;
        assistJoint.breakTorque = assistBreakForce;
        Log("Assist", "FixedJoint created.");
    }

    void DestroyAssist()
    {
        if (assistJoint != null) { Destroy(assistJoint); assistJoint = null; }
    }

    // ------------------------------------------------------------------ geometry

    /// Tool points straight down; fingers close along the horizontal object axis most aligned with the arm's tangent.
    Quaternion ComputeGraspRotation(Transform obj)
    {
        Transform rb = arm.robotBase;
        Vector3 radial = Vector3.ProjectOnPlane(obj.position - rb.position, Vector3.up);
        if (radial.sqrMagnitude < 1e-4f) radial = rb.forward;
        radial.Normalize();
        Vector3 tangent = Vector3.Cross(Vector3.up, radial);

        Vector3 best = tangent;
        float bestDot = -2f;
        foreach (var c in new[] { obj.right, -obj.right, obj.forward, -obj.forward })
        {
            Vector3 h = Vector3.ProjectOnPlane(c, Vector3.up);
            if (h.sqrMagnitude < 1e-4f) continue;
            h.Normalize();
            float d = Vector3.Dot(h, tangent);
            if (d > bestDot) { bestDot = d; best = h; }
        }
        best = Quaternion.AngleAxis(graspYawOffset, Vector3.up) * best;

        Quaternion attachWorld = Quaternion.LookRotation(Vector3.down, best);
        return attachWorld * Quaternion.Inverse(gripAttachPoint.localRotation);
    }

    /// tool0 position that puts the grip attach point on tcpPoint.
    Vector3 Flange(Vector3 tcpPoint, Quaternion tool0Rot) => tcpPoint - tool0Rot * gripAttachPoint.localPosition;

    float FlangeError(Vector3 target) => Vector3.Distance(gripAttachPoint.parent.position, target);

    float ObjectToTool() => Vector3.Distance(WorldBounds(tomatoTarget).center, gripAttachPoint.position);

    static Bounds WorldBounds(Transform obj)
    {
        var cols = obj.GetComponentsInChildren<Collider>();
        if (cols.Length > 0)
        {
            Bounds b = cols[0].bounds;
            for (int i = 1; i < cols.Length; i++) b.Encapsulate(cols[i].bounds);
            return b;
        }
        var rends = obj.GetComponentsInChildren<Renderer>();
        if (rends.Length > 0)
        {
            Bounds b = rends[0].bounds;
            for (int i = 1; i < rends.Length; i++) b.Encapsulate(rends[i].bounds);
            return b;
        }
        return new Bounds(obj.position, Vector3.one * 0.05f);
    }

    /// First non-robot, non-target collider below 'around'.
    bool TryFindSurface(Vector3 around, out Vector3 point)
    {
        var hits = Physics.RaycastAll(around + Vector3.up * surfaceRayHeight, Vector3.down,
                                      surfaceRayHeight * 3f, ~0, QueryTriggerInteraction.Ignore);
        System.Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));
        foreach (var h in hits)
        {
            if (h.transform.IsChildOf(tomatoTarget)) continue;
            if (h.collider.GetComponentInParent<ArticulationBody>() != null) continue;
            point = h.point;
            return true;
        }
        point = around;
        return false;
    }

    // ------------------------------------------------------------------ debugging

    /// Lists non-robot colliders near the tool (used when a descent stops short).
    void LogBlockers()
    {
        var seen = new HashSet<string>();
        foreach (var mine in gripAttachPoint.parent.GetComponentsInChildren<Collider>())
        {
            if (mine.isTrigger) continue;
            Bounds b = mine.bounds; b.Expand(0.01f);
            foreach (var other in Physics.OverlapBox(b.center, b.extents, Quaternion.identity, ~0, QueryTriggerInteraction.Ignore))
            {
                if (other == mine || other.GetComponentInParent<ArticulationBody>() != null) continue;
                if (seen.Add(mine.name + "|" + other.name))
                    Debug.Log($"[Blockers] '{mine.name}' vs '{other.name}' | other y=[{other.bounds.min.y:F3},{other.bounds.max.y:F3}] mine bottom y={mine.bounds.min.y:F3}");
            }
        }
        if (seen.Count == 0) Debug.Log("[Blockers] no non-robot collider near the gripper.");
    }
}
