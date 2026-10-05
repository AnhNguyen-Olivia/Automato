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

    // Must be below RESET_SENTINEL (-100) in harvest_pose_server.py.
    const float ResetSentinel = -999f;

    [Header("Scene references")]
    public Transform tomatoTarget;
    public Transform placementTarget;
    [Tooltip("The base_link object of the UR10e. Must match the frame MoveIt plans in.")]
    public Transform robotBase;

    [Header("Grasp orientation")]
    [Tooltip("Point the tool straight down (approach from above). Recommended.")]
    public bool topDownGrasp = true;
    [Tooltip("Rotation about the vertical axis in degrees. Changes which way the RG2 fingers straddle the tomato. Try 0 and 90.")]
    public float graspYawDegrees = 0f;
    [Tooltip("Only used when Top Down Grasp is off. WORLD euler angles of tool0.")]
    public Vector3 graspEuler = Vector3.zero;

    [Header("Grasp point")]
    [Tooltip("Where to grip, relative to the tomato centre, along world up (metres). 0 = centre, positive = towards the top.")]
    public float graspHeightOffset = 0f;

    [Header("Tool centre point")]
    [Tooltip("The tool0 object. If empty, the parent of Grip Attach Point is used.")]
    public Transform tool0;
    [Tooltip("Measure the approach axis and TCP offset from Grip Attach Point instead of using the two values below.")]
    public bool autoTcpFromAttachPoint = true;
    [Tooltip("Distance in metres from the tool0 flange to the point between the fingertips.")]
    public float tcpOffset = 0.15f;
    [Tooltip("Axis of tool0 (in tool0's local space) that points from the flange towards the fingertips.")]
    public Vector3 toolApproachLocal = Vector3.forward;

    [Header("Approach")]
    [Tooltip("Metres above the target the tool hovers before descending / after lifting.")]
    public float approachHeight = 0.10f;

    [Header("Gripper values")]
    public float gripperOpen = 0.0f;
    public float gripperClosed = 0.6f;

    [Header("Timing")]
    [Tooltip("Seconds to wait for the ROS service before giving up on a step.")]
    public float serviceTimeout = 15f;
    [Tooltip("Seconds to let the arm settle after a trajectory finishes.")]
    public float armSettleTime = 0.3f;

    [Header("Grasping")]
    [Tooltip("Point on the gripper, between the fingertips, that held objects snap to and follow. Create an empty child under tool0 and position it there.")]
    public Transform gripAttachPoint;

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

    // NOTE: values already saved in the Inspector override these defaults.
    // Defaults below come from the /joint_states output of the URDF mimic setup
    // (finger_joint = +x -> left_inner_knuckle -x, left_inner_finger +x,
    //  right_outer_knuckle -x, right_inner_knuckle -x, right_inner_finger +x).
    // If the linkage looks deformed in Unity, flip the sign of the wrong ones.
    [Header("Gripper joints (Unity side)")]
    public List<GripperJointSetting> gripperJoints = new List<GripperJointSetting>
    {
        new GripperJointSetting { linkName = "left_outer_knuckle",  multiplier =  1f },
        new GripperJointSetting { linkName = "left_inner_knuckle",  multiplier = -1f },
        new GripperJointSetting { linkName = "left_inner_finger",   multiplier =  1f },
        new GripperJointSetting { linkName = "right_outer_knuckle", multiplier = -1f },
        new GripperJointSetting { linkName = "right_inner_knuckle", multiplier = -1f },
        new GripperJointSetting { linkName = "right_inner_finger",  multiplier =  1f },
    };
    public float gripperStiffness = 20000f;
    public float gripperDamping = 2000f;
    public float gripperForceLimit = 200f;
    public float gripperSettleTime = 1.0f;

    // URDF joint name -> child link name (which carries the ArticulationBody)
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

    // False until the ROS server has been told "Unity arm is at home".
    // Reset to false every time Play starts, so the server never keeps the old tracked pose.
    bool serverStateKnown = false;

    // ------------------------------------------------------------------ setup

    void Awake()
    {
        FindJoints();
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<HarvestPoseRequest, HarvestPoseResponse>(serviceName);
        CalibrateTcp();
    }

    Transform RobotRoot()
    {
        if (robotBase != null) return robotBase.root;
        var anchor = GameObject.Find("wrist_3_link"); // only exists on the UR10e
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
            {
                if (ab.name == kv.Value) joints[kv.Key] = ab;
            }
            foreach (var g in gripperJoints)
            {
                if (ab.name == g.linkName) gripperBodies[g.linkName] = ab;
            }
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
            // Hold the current pose instead of snapping to a target of 0.
            if (ab.dofCount > 0)
                d.target = ab.jointPosition[0] * Mathf.Rad2Deg;
            ab.xDrive = d;
            ab.useGravity = false;
        }
        Debug.Log("[HarvestPoseClient] Drives configured.");
    }

    /// Measures the approach axis and TCP offset from the Grip Attach Point, so they can't be wrong by guesswork.
    void CalibrateTcp()
    {
        if (!autoTcpFromAttachPoint) return;

        Transform tool = tool0 != null ? tool0 : (gripAttachPoint != null ? gripAttachPoint.parent : null);
        if (tool == null || gripAttachPoint == null)
        {
            Debug.LogWarning("[TCP] Can't auto-calibrate: assign Grip Attach Point (a child of tool0) and/or Tool0.");
            return;
        }

        float dist = Vector3.Distance(tool.position, gripAttachPoint.position);
        if (dist < 0.02f)
        {
            Debug.LogWarning("[TCP] Grip Attach Point is almost at tool0's origin. Move it between the fingertips.");
            return;
        }

        Vector3 local = tool.InverseTransformPoint(gripAttachPoint.position);
        toolApproachLocal = local.normalized;
        tcpOffset = dist;
        Debug.Log($"[TCP] Auto-calibrated: approach axis (tool0 local) = {toolApproachLocal}, offset = {tcpOffset:F3} m.");
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

    void OnDrawGizmosSelected()
    {
        if (gripAttachPoint == null) return;
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(gripAttachPoint.position, 0.015f);
        Transform tool = tool0 != null ? tool0 : gripAttachPoint.parent;
        if (tool != null) Gizmos.DrawLine(tool.position, gripAttachPoint.position);
    }

    // ------------------------------------------------------------------ grasp

    /// Disable/enable collisions between the held object and the whole robot.
    void IgnoreRobotCollisions(Transform obj, bool ignore)
    {
        if (robotBase == null || obj == null) return;

        var objCols = obj.GetComponentsInChildren<Collider>();
        var robotCols = robotBase.GetComponentsInChildren<Collider>();
        foreach (var a in objCols)
        {
            foreach (var b in robotCols)
            {
                if (a == null || b == null || a == b) continue;
                if (b.transform.IsChildOf(obj)) continue; // collider belongs to the object itself
                Physics.IgnoreCollision(a, b, ignore);
            }
        }
    }

    void AttachObject(Transform obj)
    {
        if (obj == null || gripAttachPoint == null)
        {
            Debug.LogWarning("[Grasp] Missing object or gripAttachPoint, can't attach.");
            return;
        }

        // If the gripper isn't actually around the tomato, it will look like the tomato floats in front of it.
        float gap = Vector3.Distance(gripAttachPoint.position, obj.position);
        float expected = Mathf.Abs(graspHeightOffset);
        if (gap > expected + 0.03f)
        {
            Debug.LogWarning($"[Grasp] The tomato is {gap * 100f:F1} cm from the grip point (expected about {expected * 100f:F1} cm). " +
                             "The gripper is not where the tomato is: check the grasp orientation, TCP offset and Grip Attach Point.");
        }

        grabbedRb = obj.GetComponent<Rigidbody>();
        if (grabbedRb != null)
        {
            // Kinematic while held so physics doesn't fight the parent transform.
            grabbedRb.isKinematic = true;
        }

        // The held tomato must not push the arm's articulation bodies around.
        IgnoreRobotCollisions(obj, true);

        grabbedOriginalParent = obj.parent;
        obj.SetParent(gripAttachPoint, true); // keep world position, then it follows the gripper
        isHolding = true;
        Debug.Log($"[Grasp] '{obj.name}' attached to gripper.");
    }

    void DetachObject(Transform obj)
    {
        if (obj == null) return;

        obj.SetParent(grabbedOriginalParent, true); // keep current world position on release
        IgnoreRobotCollisions(obj, false);
        if (grabbedRb != null)
        {
            grabbedRb.isKinematic = false; // let it fall / settle under physics again
        }
        isHolding = false;
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
        StartCoroutine(RunHarvestSequence());
    }

    /// Tool orientation used for every step of the sequence.
    Quaternion GraspRotation()
    {
        if (!topDownGrasp) return Quaternion.Euler(graspEuler);

        // Rotate tool0 so that its approach axis points straight down, then spin about the vertical.
        Quaternion pointDown = Quaternion.FromToRotation(toolApproachLocal.normalized, Vector3.down);
        return Quaternion.AngleAxis(graspYawDegrees, Vector3.up) * pointDown;
    }

    /// Converts a point between the fingertips into the tool0 flange position.
    Vector3 Flange(Vector3 tcpPoint, Quaternion rot)
    {
        return tcpPoint - (rot * toolApproachLocal.normalized) * tcpOffset;
    }

    /// Tells the ROS server that the Unity arm is at its home pose, so it forgets the old tracked state.
    IEnumerator SendReset()
    {
        bool done = false;
        HarvestPoseResponse result = null;

        var request = new HarvestPoseRequest();
        request.target_pose = new PoseMsg();
        request.gripper_position = ResetSentinel;

        ros.SendServiceMessage<HarvestPoseResponse>(serviceName, request, (HarvestPoseResponse response) =>
        {
            result = response;
            done = true;
        });

        float t0 = Time.realtimeSinceStartup;
        yield return new WaitUntil(() => done || Time.realtimeSinceStartup - t0 > serviceTimeout);

        if (!done)
        {
            Debug.LogError("[Reset] Service timed out. Is the ROS side running?");
            yield break;
        }
        if (result == null || !result.success)
        {
            Debug.LogError($"[Reset] FAILED: {result?.message}");
            yield break;
        }

        serverStateKnown = true;
        Debug.Log($"[Reset] {result.message}");
    }

    IEnumerator RunHarvestSequence()
    {
        sequenceRunning = true;

        if (tomatoTarget == null || placementTarget == null || robotBase == null)
        {
            Debug.LogError("Assign tomatoTarget, placementTarget and robotBase on the Publisher.");
            sequenceRunning = false;
            yield break;
        }

        // First run after pressing Play: the arm is at home, so make the server forget its old tracked pose.
        if (!serverStateKnown)
        {
            yield return SendReset();
            if (!serverStateKnown)
            {
                sequenceRunning = false;
                yield break;
            }
        }

        // Keep the tomato still while the fingers approach, so they can't knock it away.
        Rigidbody tomatoRb = tomatoTarget.GetComponent<Rigidbody>();
        bool tomatoWasKinematic = tomatoRb != null && tomatoRb.isKinematic;
        if (tomatoRb != null) tomatoRb.isKinematic = true;

        Quaternion grasp = GraspRotation();
        Vector3 up = Vector3.up * approachHeight;

        Vector3 tomatoPoint = tomatoTarget.position + Vector3.up * graspHeightOffset;
        Vector3 tomato = Flange(tomatoPoint, grasp);
        Vector3 basket = Flange(placementTarget.position, grasp);

        var steps = new (Vector3 pos, float grip, string label)[]
        {
            (tomato + up, gripperOpen,   "Hover over tomato"),
            (tomato,      gripperOpen,   "Descend to tomato"),
            (tomato,      gripperClosed, "Grasp tomato"),
            (tomato + up, gripperClosed, "Lift tomato"),
            (basket + up, gripperClosed, "Move above basket"),
            (basket,      gripperClosed, "Lower into basket"),
            (basket,      gripperOpen,   "Release tomato"),
        };

        bool completed = true;
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
            else if (s.label == "Release tomato") DetachObject(tomatoTarget);
        }

        // If we gave up before the tomato was picked, put its physics state back.
        if (!completed && !isHolding && tomatoRb != null) tomatoRb.isKinematic = tomatoWasKinematic;

        if (completed) Debug.Log("Harvest sequence complete.");
        sequenceRunning = false;
    }

    IEnumerator SendAndAnimate(Vector3 targetPosition, Quaternion targetRotation, float gripperPosition, string stepLabel)
    {
        lastStepOk = false;
        bool done = false;
        HarvestPoseResponse result = null;

        // Express the target in the robot base's frame, then convert Unity -> ROS
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
        yield return new WaitForSeconds(armSettleTime);

        ApplyGripper(gripperPosition);
        yield return new WaitForSeconds(gripperSettleTime);

        Debug.Log($"[{stepLabel}] Animation complete.");
        lastStepOk = true;
    }

    void SetArmTargets(ArticulationBody[] cols, double[] from, double[] to, float a)
    {
        for (int i = 0; i < cols.Length && i < from.Length && i < to.Length; i++)
        {
            if (cols[i] == null) continue;
            double rad = from[i] + (to[i] - from[i]) * a;
            var drive = cols[i].xDrive;
            drive.target = (float)(rad * Mathf.Rad2Deg);
            cols[i].xDrive = drive;
        }
    }

    IEnumerator PlayTrajectory(RosMessageTypes.Trajectory.JointTrajectoryMsg trajectory)
    {
        if (joints.Count == 0) FindJoints();

        // Resolve each trajectory column to an ArticulationBody once, by name.
        var cols = new ArticulationBody[trajectory.joint_names.Length];
        for (int i = 0; i < cols.Length; i++)
        {
            string jn = trajectory.joint_names[i];
            if (!joints.TryGetValue(jn, out cols[i]))
            {
                // Gripper joints are driven separately by ApplyGripper, so don't warn about those.
                if (!jn.ToLower().Contains("finger") && !jn.ToLower().Contains("knuckle"))
                    Debug.LogWarning($"[PlayTrajectory] No Unity joint for '{jn}', skipping it.");
            }
        }

        var pts = trajectory.points;
        int n = pts.Length;
        if (n == 0) yield break;

        // Does Unity's arm actually start where MoveIt thinks it does?
        float worstDiff = 0f;
        string worstJoint = "";
        for (int i = 0; i < cols.Length; i++)
        {
            if (cols[i] == null || cols[i].dofCount == 0 || i >= pts[0].positions.Length) continue;
            float diff = Mathf.Abs(cols[i].jointPosition[0] - (float)pts[0].positions[i]);
            if (diff > worstDiff) { worstDiff = diff; worstJoint = trajectory.joint_names[i]; }
        }
        if (worstDiff > 0.05f)
        {
            Debug.LogWarning($"[PlayTrajectory] Unity's arm is {worstDiff:F2} rad away from where the plan starts (worst joint: {worstJoint}). " +
                             "Unity and MoveIt disagree about the arm pose, so it will snap at the start of this move.");
        }

        float[] times = new float[n];
        for (int k = 0; k < n; k++)
            times[k] = (float)(pts[k].time_from_start.sec + pts[k].time_from_start.nanosec / 1e9);

        // Interpolate between points every frame, timed against the clock.
        float start = Time.time;
        int seg = 0;
        while (true)
        {
            float t = Time.time - start;
            bool finished = t >= times[n - 1];
            if (finished) t = times[n - 1];

            if (n == 1)
            {
                SetArmTargets(cols, pts[0].positions, pts[0].positions, 0f);
            }
            else
            {
                while (seg < n - 2 && times[seg + 1] < t) seg++;
                float span = times[seg + 1] - times[seg];
                float a = span > 1e-6f ? Mathf.Clamp01((t - times[seg]) / span) : 1f;
                SetArmTargets(cols, pts[seg].positions, pts[seg + 1].positions, a);
            }

            if (finished) break;
            yield return null;
        }
    }
}
