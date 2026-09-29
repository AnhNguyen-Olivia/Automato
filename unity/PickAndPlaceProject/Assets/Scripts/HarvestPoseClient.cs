using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Ur10eRg2Moveit;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

public class HarvestPoseClient : MonoBehaviour
{
    ROSConnection ros;
    const string serviceName = "harvest_pose";

    [Header("Scene references")]
    public Transform tomatoTarget;
    public Transform placementTarget;
    [Tooltip("The base_link object of the UR10e. Must match the frame MoveIt plans in.")]
    public Transform robotBase;
    [Tooltip("Optional. If set, obstacles are re-published to MoveIt every time the button is clicked.")]
    public PlanningSceneSync sceneSync;

    [Header("Tool frame (set up ONCE)")]
    [Tooltip("Empty child of tool0 placed between the fingertips.\n" +
             "Convention: local +Z points from the flange toward the fingertips (approach axis),\n" +
             "local +Y is the axis the fingers open/close along.\n" +
             "Its position and rotation replace the old tcpOffset / toolApproachLocal / graspEuler.")]
    public Transform gripAttachPoint;
    [Tooltip("Extra yaw (degrees) around vertical. Use 90 if your fingers close along X instead of Y.")]
    public float graspYawOffset = 0f;

    [Header("Approach / placing")]
    [Tooltip("Metres above the target the tool hovers before descending / after lifting.")]
    public float approachHeight = 0.10f;
    [Tooltip("Gap between the bottom of the object and the surface when released (metres).")]
    public float placeClearance = 0.005f;
    [Tooltip("How high above the placement target the surface-detection ray starts.")]
    public float surfaceRayHeight = 1.0f;
    [Tooltip("How far the finger pads reach below the grip attach point (metres). Used so the pads never touch the table.")]
    public float fingertipDrop = 0.02f;
    [Tooltip("Vertical moves are split into pieces of this length so the tool travels in a straight line (metres).")]
    public float verticalStepSize = 0.03f;

    [Header("Gripper values")]
    public float gripperOpen = 0.0f;
    public float gripperClosed = 0.6f;

    [Header("Timing")]
    [Tooltip("Seconds to wait for the ROS service before giving up on a step.")]
    public float serviceTimeout = 15f;
    [Tooltip("1 = play the MoveIt timing as-is. >1 = slower, easier for the drives to track.")]
    public float trajectoryTimeScale = 1f;
    [Tooltip("Max seconds to wait for the arm to actually reach the final pose.")]
    public float armSettleTimeout = 6f;
    public float settleToleranceDeg = 0.5f;
    public float settleVelocity = 0.02f;
    [Tooltip("Max seconds to wait for the fingers to finish moving / stall on the object.")]
    public float gripperSettleTime = 1.5f;

    Rigidbody grabbedRb;
    Transform grabbedOriginalParent;
    bool isHolding = false;

    [Header("Joint drive tuning")]
    public float driveStiffness = 100000f;
    public float driveDamping = 10000f;
    public float driveForceLimit = 10000f;

    [System.Serializable]
    public class GripperJointSetting
    {
        public string linkName;      // child link carrying the ArticulationBody
        public float multiplier = 1; // the URDF <mimic> multiplier relative to finger_joint
    }

    [Header("Gripper joints (Unity side)")]
    [Tooltip("Verify each multiplier against the <mimic> tags in your gripper URDF.")]
    public List<GripperJointSetting> gripperJoints = new List<GripperJointSetting>
    {
        new GripperJointSetting { linkName = "left_outer_knuckle",  multiplier =  1f },
        new GripperJointSetting { linkName = "left_inner_knuckle",  multiplier =  1f },
        new GripperJointSetting { linkName = "left_inner_finger",   multiplier = -1f },
        new GripperJointSetting { linkName = "right_outer_knuckle", multiplier =  1f },
        new GripperJointSetting { linkName = "right_inner_knuckle", multiplier =  1f },
        new GripperJointSetting { linkName = "right_inner_finger",  multiplier = -1f },
    };
    public float gripperStiffness = 20000f;
    public float gripperDamping = 2000f;
    public float gripperForceLimit = 200f;

    static readonly Dictionary<string, string> jointToLink = new Dictionary<string, string>
    {
        { "shoulder_pan_joint",  "shoulder_link"  },
        { "shoulder_lift_joint", "upper_arm_link" },
        { "elbow_joint",         "forearm_link"   },
        { "wrist_1_joint",       "wrist_1_link"   },
        { "wrist_2_joint",       "wrist_2_link"   },
        { "wrist_3_joint",       "wrist_3_link"   },
    };

    readonly Dictionary<string, ArticulationBody> joints = new Dictionary<string, ArticulationBody>();
    readonly Dictionary<string, ArticulationBody> gripperBodies = new Dictionary<string, ArticulationBody>();

    bool sequenceRunning = false;
    bool lastStepOk = false;

    // ------------------------------------------------------------------ setup

    void Awake() { FindJoints(); }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<HarvestPoseRequest, HarvestPoseResponse>(serviceName);
    }

    Transform RobotRoot()
    {
        if (robotBase != null) return robotBase.root;
        var anchor = GameObject.Find("wrist_3_link");
        return anchor != null ? anchor.transform.root : null;
    }

    [ContextMenu("Find Joints")]
    void FindJoints()
    {
        joints.Clear();
        gripperBodies.Clear();

        Transform robotRoot = RobotRoot();
        if (robotRoot == null)
        {
            Debug.LogError("[HarvestPoseClient] Could not locate the UR10e. Assign robotBase or make sure wrist_3_link is in the scene and active.");
            return;
        }

        foreach (var ab in robotRoot.GetComponentsInChildren<ArticulationBody>())
        {
            foreach (var kv in jointToLink)
                if (ab.name == kv.Value) joints[kv.Key] = ab;
            foreach (var g in gripperJoints)
                if (ab.name == g.linkName) gripperBodies[g.linkName] = ab;
        }

        foreach (var kv in jointToLink)
            if (!joints.ContainsKey(kv.Key))
                Debug.LogError($"[HarvestPoseClient] No ArticulationBody found for {kv.Key} (link {kv.Value}).");

        foreach (var g in gripperJoints)
            if (!gripperBodies.ContainsKey(g.linkName))
                Debug.LogWarning($"[HarvestPoseClient] Gripper link '{g.linkName}' not found.");

        ConfigureDrives();
    }

    void ConfigureDrives()
    {
        foreach (var ab in joints.Values)
        {
            var d = ab.xDrive;
            d.stiffness = driveStiffness;
            d.damping = driveDamping;
            d.forceLimit = driveForceLimit;
            if (ab.dofCount > 0)
                d.target = ab.jointPosition[0] * Mathf.Rad2Deg;
            ab.xDrive = d;
            ab.useGravity = false;
        }
        Debug.Log("[HarvestPoseClient] Drives configured.");
    }

    void ApplyGripper(float radians)
    {
        foreach (var g in gripperJoints)
        {
            if (!gripperBodies.TryGetValue(g.linkName, out var ab)) continue;
            var d = ab.xDrive;
            d.stiffness = gripperStiffness;
            d.damping = gripperDamping;
            d.forceLimit = gripperForceLimit;
            d.target = radians * g.multiplier * Mathf.Rad2Deg;
            ab.xDrive = d;
        }
    }

    // ------------------------------------------------------- scene awareness

    /// Combined world bounds of every collider (or renderer as fallback) under obj.
    static Bounds GetWorldBounds(Transform obj)
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

    /// Casts a ray straight down and returns the first real surface (ignores the robot and the held object).
    bool TryFindSurface(Vector3 around, out Vector3 point)
    {
        Vector3 origin = around + Vector3.up * surfaceRayHeight;
        var hits = Physics.RaycastAll(origin, Vector3.down, surfaceRayHeight * 3f, ~0, QueryTriggerInteraction.Ignore);
        System.Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));

        foreach (var h in hits)
        {
            if (h.transform.IsChildOf(tomatoTarget)) continue;                 // the object itself
            if (h.collider.GetComponentInParent<ArticulationBody>() != null) continue; // the robot
            point = h.point;
            return true;
        }
        point = around;
        return false;
    }

    /// Top-down grasp; yaw is snapped to the object's own faces. No manual Euler angles needed.
    Quaternion ComputeGraspRotation(Transform obj)
    {
        Vector3 radial = Vector3.ProjectOnPlane(obj.position - robotBase.position, Vector3.up);
        if (radial.sqrMagnitude < 1e-4f) radial = robotBase.forward;
        radial.Normalize();
        Vector3 tangent = Vector3.Cross(Vector3.up, radial);

        // Pick the object's horizontal axis closest to the tangent so the wrist twists as little as possible.
        Vector3[] candidates = { obj.right, -obj.right, obj.forward, -obj.forward };
        Vector3 best = tangent;
        float bestDot = -2f;
        foreach (var c in candidates)
        {
            Vector3 h = Vector3.ProjectOnPlane(c, Vector3.up);
            if (h.sqrMagnitude < 1e-4f) continue;
            h.Normalize();
            float d = Vector3.Dot(h, tangent);
            if (d > bestDot) { bestDot = d; best = h; }
        }
        best = Quaternion.AngleAxis(graspYawOffset, Vector3.up) * best;

        // Wanted world rotation of the attach point: +Z down, +Y along `best`.
        Quaternion attachWorld = Quaternion.LookRotation(Vector3.down, best);
        // tool0 = attachWorld * inverse(attach's rotation relative to tool0)
        return attachWorld * Quaternion.Inverse(gripAttachPoint.localRotation);
    }

    /// Converts a point between the fingertips into the tool0 flange position.
    Vector3 Flange(Vector3 tcpPoint, Quaternion tool0Rot)
    {
        return tcpPoint - (tool0Rot * gripAttachPoint.localPosition);
    }

    // ------------------------------------------------------------------ grasp

    /// While the object is held, its collider must not fight the robot's colliders (causes jitter / popping).
    void SetHeldCollisions(Transform obj, bool ignore)
    {
        Transform root = RobotRoot();
        if (root == null) return;

        var robotCols = root.GetComponentsInChildren<Collider>();
        foreach (var oc in obj.GetComponentsInChildren<Collider>())
            foreach (var rc in robotCols)
                if (oc != rc) Physics.IgnoreCollision(oc, rc, ignore);
    }

    void AttachObject(Transform obj)
    {
        grabbedRb = obj.GetComponent<Rigidbody>();
        if (grabbedRb != null) grabbedRb.isKinematic = true;

        grabbedOriginalParent = obj.parent;
        obj.SetParent(gripAttachPoint, true); // keep world pose, no teleport
        isHolding = true;
        SetHeldCollisions(obj, true);
        Debug.Log($"[Grasp] '{obj.name}' attached to gripper.");
    }

    void DetachObject(Transform obj)
    {
        obj.SetParent(grabbedOriginalParent, true);
        if (grabbedRb != null)
        {
            grabbedRb.isKinematic = false;
            grabbedRb.velocity = Vector3.zero;        // Unity 6: linearVelocity
            grabbedRb.angularVelocity = Vector3.zero;
        }
        isHolding = false;
        SetHeldCollisions(obj, false);
        Debug.Log($"[Release] '{obj.name}' detached from gripper.");
    }

    // --------------------------------------------------------------- sequence

    public void OnHarvestButtonClicked()
    {
        if (sequenceRunning)
        {
            Debug.LogWarning("Harvest sequence already running, ignoring click.");
            return;
        }
        if (sceneSync != null) sceneSync.PublishAll(); // make sure MoveIt knows the table before planning
        StartCoroutine(RunHarvestSequence());
    }

    /// Adds waypoints on a straight line from `from` to `to`, one every `verticalStepSize` metres.
    /// Each short segment is planned separately, so the tool tip cannot swing sideways.
    void AddLinear(List<(Vector3 pos, float grip, string label)> steps,
                   Vector3 from, Vector3 to, float grip, string label)
    {
        int n = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(from, to) / verticalStepSize));
        for (int i = 1; i <= n; i++)
            steps.Add((Vector3.Lerp(from, to, i / (float)n), grip, $"{label} {i}/{n}"));
    }

    IEnumerator RunHarvestSequence()
    {
        sequenceRunning = true;

        if (tomatoTarget == null || placementTarget == null || robotBase == null || gripAttachPoint == null)
        {
            Debug.LogError("Assign tomatoTarget, placementTarget, robotBase and gripAttachPoint.");
            sequenceRunning = false;
            yield break;
        }

        // 1. Measure the object instead of hard-coding anything.
        Bounds tomatoBounds = GetWorldBounds(tomatoTarget);
        Vector3 pickPoint = tomatoBounds.center;
        float halfHeight = tomatoBounds.extents.y;

        // Never let the finger pads go below the real table surface.
        if (TryFindSurface(pickPoint, out Vector3 tableAtTomato))
            pickPoint.y = Mathf.Max(pickPoint.y, tableAtTomato.y + fingertipDrop + 0.003f);

        // 2. Find the real surface under the placement target (table, basket floor, ...).
        Vector3 placePoint = placementTarget.position;
        if (TryFindSurface(placementTarget.position, out Vector3 surface))
        {
            placePoint = surface + Vector3.up * (halfHeight + placeClearance);
            Debug.Log($"[Place] Surface found at y={surface.y:F3}, releasing with object centre at y={placePoint.y:F3}");
        }
        else
        {
            placePoint = placementTarget.position + Vector3.up * (halfHeight + placeClearance);
            Debug.LogWarning("[Place] No surface found under placementTarget; using its position + object half-height.");
        }

        // 3. Grasp orientation derived from the object and the robot base.
        Quaternion grasp = ComputeGraspRotation(tomatoTarget);
        Vector3 offsetWorld   = grasp * gripAttachPoint.localPosition;                        // flange -> fingertips, in the planned pose
	Vector3 approachWorld = grasp * (gripAttachPoint.localRotation * Vector3.forward);    // attach +Z, in the planned pose
	Vector3 cubeLocal     = robotBase.InverseTransformPoint(pickPoint);
	Debug.Log($"[Check2] offset={offsetWorld}  approach={approachWorld}  cube horizontal dist={new Vector2(cubeLocal.x, cubeLocal.z).magnitude:F3} m");
        Vector3 up = Vector3.up * approachHeight;

        Vector3 pick = Flange(pickPoint, grasp);
        Vector3 place = Flange(placePoint, grasp);
            
        if (Vector3.Dot(gripAttachPoint.localPosition.normalized,
                gripAttachPoint.localRotation * Vector3.forward) < 0.99f)
        Debug.LogWarning("gripAttachPoint +Z does not point from the flange toward the fingertips. Fix its local rotation.");

        // Free-space moves only for the hover legs; every vertical leg is a straight line of short segments.
        var steps = new List<(Vector3 pos, float grip, string label)>();
        steps.Add((pick + up, gripperOpen, "Hover over tomato"));
        AddLinear(steps, pick + up, pick, gripperOpen, "Descend");
        steps.Add((pick, gripperClosed, "Grasp tomato"));          // label must stay exact (AttachObject check)
        AddLinear(steps, pick, pick + up, gripperClosed, "Lift");
        steps.Add((place + up, gripperClosed, "Move above basket"));
        AddLinear(steps, place + up, place, gripperClosed, "Lower");
        steps.Add((place, gripperOpen, "Release tomato"));         // label must stay exact (DetachObject check)
        AddLinear(steps, place, place + up, gripperOpen, "Retreat");

        bool completed = true;

        Debug.Log($"[Check] base y={robotBase.position.y:F3}  cubeCentre y={pickPoint.y:F3}  " +
                  $"attach parent='{gripAttachPoint.parent.name}'  attach localPos={gripAttachPoint.localPosition}  " +
                  $"attach +Z in world={gripAttachPoint.forward}");

        foreach (var s in steps)
        {
            yield return SendAndAnimate(s.pos, grasp, s.grip, s.label);
            if (!lastStepOk)
            {
                Debug.LogError($"Sequence aborted at step: {s.label}");
                completed = false;
                break;
            }

            if (s.label == "Grasp tomato") AttachObject(tomatoTarget);
            else if (s.label == "Release tomato") DetachObject(tomatoTarget); // only after arm AND fingers have settled
        }

        if (completed) Debug.Log("Harvest sequence complete.");
        sequenceRunning = false;
    }

    IEnumerator SendAndAnimate(Vector3 targetPosition, Quaternion targetRotation, float gripperPosition, string stepLabel)
    {
        lastStepOk = false;
        bool done = false;
        HarvestPoseResponse result = null;

        Vector3 localPos = robotBase.InverseTransformPoint(targetPosition);
        Quaternion localRot = Quaternion.Inverse(robotBase.rotation) * targetRotation;

        HarvestPoseRequest request = new HarvestPoseRequest();
        request.target_pose = new PoseMsg(
            new PointMsg(localPos.z, -localPos.x, localPos.y),
            new QuaternionMsg(-localRot.z, localRot.x, -localRot.y, localRot.w)
        );
        request.gripper_position = gripperPosition;

        Debug.Log($"[{stepLabel}] Sending request... ROS pos=({localPos.z:F3}, {-localPos.x:F3}, {localPos.y:F3})");

        ros.SendServiceMessage<HarvestPoseResponse>(serviceName, request, (HarvestPoseResponse response) =>
        {
            result = response;
            done = true;
        });

        float t0 = Time.realtimeSinceStartup;
        yield return new WaitUntil(() => done || Time.realtimeSinceStartup - t0 > serviceTimeout);

        if (!done)
        {
            Debug.LogError($"[{stepLabel}] Service timed out after {serviceTimeout:F0}s. Is the ROS side running?");
            yield break;
        }
        if (result == null || !result.success)
        {
            Debug.LogError($"[{stepLabel}] FAILED: {result?.message}");
            yield break;
        }

        Debug.Log($"[{stepLabel}] Plan OK, animating {result.trajectory.points.Length} points...");
        yield return PlayTrajectory(result.trajectory);

        // Wait until the arm has REALLY arrived (the drives lag behind the commanded targets),
        // otherwise the gripper opens while the arm is still moving and flings the object.
        yield return WaitForArmSettled();

        ApplyGripper(gripperPosition);
        yield return WaitForGripperSettled();

        Debug.Log($"[{stepLabel}] Animation complete.");
        lastStepOk = true;
    }

    // ------------------------------------------------------------- settling

    bool ArmIsSettled(out string reason)
    {
        foreach (var kvp in joints)
        {
            var ab = kvp.Value;
            if (ab.dofCount == 0) continue;

            float posDeg = ab.jointPosition[0] * Mathf.Rad2Deg;
            float posErr = Mathf.Abs(posDeg - ab.xDrive.target);
            float vel = Mathf.Abs(ab.jointVelocity[0]);

            if (posErr > settleToleranceDeg)
            {
                reason = $"{kvp.Key} pos error ({posErr:F2}° > {settleToleranceDeg:F2}°)";
                return false;
            }
            if (vel > settleVelocity)
            {
                reason = $"{kvp.Key} vel too high ({vel:F3} > {settleVelocity:F3})";
                return false;
            }
        }
        reason = "OK";
        return true;
    }

    IEnumerator WaitForArmSettled()
    {
        float start = Time.time, stableSince = -1f;
        string lastReason = "";

        while (Time.time - start < armSettleTimeout)
        {
            if (ArmIsSettled(out lastReason))
            {
                if (stableSince < 0) stableSince = Time.time;
                if (Time.time - stableSince >= 0.15f) yield break;
            }
            else
            {
                stableSince = -1f;
            }
            yield return null;
        }
        Debug.LogWarning($"[Settle] Arm timeout after {armSettleTimeout}s. Last issue: {lastReason}");
    }

    IEnumerator WaitForGripperSettled()
    {
        yield return new WaitForSeconds(0.2f); // let the new drive target take effect
        float start = Time.time, stableSince = -1f;
        while (Time.time - start < gripperSettleTime)
        {
            bool still = true;
            foreach (var ab in gripperBodies.Values)
                if (ab.dofCount > 0 && Mathf.Abs(ab.jointVelocity[0]) > settleVelocity) { still = false; break; }

            if (still)
            {
                if (stableSince < 0) stableSince = Time.time;
                if (Time.time - stableSince >= 0.15f) yield break; // stopped moving or stalled on the object
            }
            else stableSince = -1f;
            yield return null;
        }
    }

    // ------------------------------------------------------------ trajectory

    IEnumerator PlayTrajectory(RosMessageTypes.Trajectory.JointTrajectoryMsg trajectory)
    {
        if (joints.Count == 0) FindJoints();

        var cols = new ArticulationBody[trajectory.joint_names.Length];
        for (int i = 0; i < cols.Length; i++)
        {
            string jn = trajectory.joint_names[i];
            if (!joints.TryGetValue(jn, out cols[i]))
            {
                if (!jn.ToLower().Contains("finger") && !jn.ToLower().Contains("knuckle"))
                    Debug.LogWarning($"[PlayTrajectory] No Unity joint for '{jn}', skipping it.");
            }
        }

        for (int p = 0; p < trajectory.points.Length; p++)
        {
            var point = trajectory.points[p];
            float pointTime = (float)(point.time_from_start.sec + point.time_from_start.nanosec / 1e9);

            if (p == 0)
            {
                for (int i = 0; i < point.positions.Length && i < cols.Length; i++)
                {
                    if (cols[i] == null) continue;
                    var drive = cols[i].xDrive;
                    drive.target = (float)(point.positions[i] * Mathf.Rad2Deg);
                    cols[i].xDrive = drive;
                }
                continue;
            }

            var prevPoint = trajectory.points[p - 1];
            float prevTime = (float)(prevPoint.time_from_start.sec + prevPoint.time_from_start.nanosec / 1e9);
            float segmentDuration = Mathf.Max((pointTime - prevTime) * trajectoryTimeScale, 0.02f);

            float segStart = Time.time;
            while (Time.time - segStart < segmentDuration)
            {
                float t = (Time.time - segStart) / segmentDuration;
                for (int i = 0; i < point.positions.Length && i < cols.Length; i++)
                {
                    if (cols[i] == null) continue;
                    float prevAngle = (float)(prevPoint.positions[i] * Mathf.Rad2Deg);
                    float nextAngle = (float)(point.positions[i] * Mathf.Rad2Deg);
                    var drive = cols[i].xDrive;
                    drive.target = Mathf.Lerp(prevAngle, nextAngle, t);
                    cols[i].xDrive = drive;
                }
                yield return null;
            }

            // Make sure the very last point is commanded exactly.
            if (p == trajectory.points.Length - 1)
            {
                for (int i = 0; i < point.positions.Length && i < cols.Length; i++)
                {
                    if (cols[i] == null) continue;
                    var drive = cols[i].xDrive;
                    drive.target = (float)(point.positions[i] * Mathf.Rad2Deg);
                    cols[i].xDrive = drive;
                }
            }
        }
    }
}
