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

    [Header("Grasp orientation")]
    [Tooltip("WORLD euler angles of tool0 when the gripper points at the tomato. Read them from tool0 in the Inspector.")]
    public Vector3 graspEuler = Vector3.zero;

    [Header("Tool centre point")]
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

    [Header("Gripper joints (Unity side)")]
    [Tooltip("Verify each multiplier against the <mimic> tags in your gripper URDF.")]
    public List<GripperJointSetting> gripperJoints = new List<GripperJointSetting>
    {
        new GripperJointSetting { linkName = "left_outer_knuckle",  multiplier =  1f },
        new GripperJointSetting { linkName = "left_inner_knuckle",  multiplier =  1f },  // unverified
        new GripperJointSetting { linkName = "left_inner_finger",   multiplier = -1f },  // unverified
        new GripperJointSetting { linkName = "right_outer_knuckle", multiplier =  1f },  // unverified
        new GripperJointSetting { linkName = "right_inner_knuckle", multiplier =  1f },  // unverified
        new GripperJointSetting { linkName = "right_inner_finger",  multiplier = -1f },  // unverified
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

    // ------------------------------------------------------------------ setup

    void Awake()
    {
        FindJoints();
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<HarvestPoseRequest, HarvestPoseResponse>(serviceName);
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

    // ------------------------------------------------------------------ grasp

    void AttachObject(Transform obj)
    {
        if (obj == null || gripAttachPoint == null)
        {
            Debug.LogWarning("[Grasp] Missing object or gripAttachPoint, can't attach.");
            return;
        }

        grabbedRb = obj.GetComponent<Rigidbody>();
        if (grabbedRb != null)
        {
            // Kinematic while held so physics doesn't fight the parent transform
            // or make it slip out of the fingers.
            grabbedRb.isKinematic = true;
        }

        grabbedOriginalParent = obj.parent;
        obj.SetParent(gripAttachPoint, true); // keep world position, then it follows the gripper
        isHolding = true;
        Debug.Log($"[Grasp] '{obj.name}' attached to gripper.");
    }

    void DetachObject(Transform obj)
    {
        if (obj == null) return;

        obj.SetParent(grabbedOriginalParent, true); // keep current world position on release
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

    /// Converts a point between the fingertips into the tool0 flange position.
    Vector3 Flange(Vector3 tcpPoint, Quaternion rot)
    {
        return tcpPoint - (rot * toolApproachLocal.normalized) * tcpOffset;
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

        Quaternion grasp = Quaternion.Euler(graspEuler);
        Vector3 up = Vector3.up * approachHeight;

        Vector3 tomato = Flange(tomatoTarget.position, grasp);
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

        // Time against the clock, not per-point waits, so frame rounding doesn't accumulate.
        float start = Time.time;

        foreach (var point in trajectory.points)
        {
            float pointTime = (float)(point.time_from_start.sec + point.time_from_start.nanosec / 1e9);

            for (int i = 0; i < point.positions.Length && i < cols.Length; i++)
            {
                if (cols[i] == null) continue;
                var drive = cols[i].xDrive;
                drive.target = (float)(point.positions[i] * Mathf.Rad2Deg);
                cols[i].xDrive = drive;
            }

            if (Time.time - start < pointTime)
                yield return new WaitUntil(() => Time.time - start >= pointTime);
        }
    }
}
