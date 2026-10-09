using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Trajectory;

/// Owns the UR10e ArticulationBodies: drive setup, trajectory playback, "has the arm stopped moving?".
public class ArmController : MonoBehaviour
{
    [Tooltip("The base_link object of the UR10e. Must match the frame MoveIt plans in.")]
    public Transform robotBase;

    [Header("Drives")]
    public float stiffness = 100000f;
    public float damping = 3000f;
    public float forceLimit = 10000f;

    [Header("Playback / settling")]
    public float timeScale = 1f;
    public float settleTimeout = 3f;
    public float settleToleranceDeg = 0.5f;
    public float settleVelocity = 0.06f;

    static readonly Dictionary<string, string> JointToLink = new Dictionary<string, string>
    {
        { "shoulder_pan_joint",  "shoulder_link"  },
        { "shoulder_lift_joint", "upper_arm_link" },
        { "elbow_joint",         "forearm_link"   },
        { "wrist_1_joint",       "wrist_1_link"   },
        { "wrist_2_joint",       "wrist_2_link"   },
        { "wrist_3_joint",       "wrist_3_link"   },
    };

    readonly Dictionary<string, ArticulationBody> joints = new Dictionary<string, ArticulationBody>();

    public Transform Root => robotBase != null ? robotBase.root : null;

    void Awake() { FindJoints(); }

    [ContextMenu("Find Joints")]
    public void FindJoints()
    {
        joints.Clear();
        if (Root == null) { Debug.LogError("[Arm] Assign robotBase."); return; }

        foreach (var ab in Root.GetComponentsInChildren<ArticulationBody>())
            foreach (var kv in JointToLink)
                if (ab.name == kv.Value) joints[kv.Key] = ab;

        foreach (var kv in JointToLink)
            if (!joints.ContainsKey(kv.Key)) Debug.LogError($"[Arm] Missing ArticulationBody for {kv.Key} ({kv.Value}).");

        foreach (var ab in joints.Values)
        {
            var d = ab.xDrive;
            d.stiffness = stiffness; d.damping = damping; d.forceLimit = forceLimit;
            if (ab.dofCount > 0) d.target = ab.jointPosition[0] * Mathf.Rad2Deg; // hold current pose
            ab.xDrive = d;
            ab.useGravity = false;
        }
    }

    // ---------------------------------------------------------------- playback

    /// Cubic Hermite between trajectory points on a single clock (linear if no velocities are given).
    public IEnumerator Play(JointTrajectoryMsg traj)
    {
        if (joints.Count == 0) FindJoints();

        int n = traj.joint_names.Length, P = traj.points.Length;
        var bodies = new ArticulationBody[n];
        for (int i = 0; i < n; i++)
            if (!joints.TryGetValue(traj.joint_names[i], out bodies[i])) Debug.LogWarning($"[Arm] No Unity joint for '{traj.joint_names[i]}', skipped.");
        if (P == 0) yield break;

        float scale = Mathf.Max(timeScale, 0.01f);
        var times = new float[P];
        bool hasVel = true;
        for (int p = 0; p < P; p++)
        {
            var pt = traj.points[p];
            times[p] = (float)(pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9) * scale;
            if (pt.velocities == null || pt.velocities.Length < n) hasVel = false;
        }

        SetTargets(bodies, traj.points[0].positions);
        if (P < 2) yield break;

        float start = Time.time, total = times[P - 1];
        int seg = 1;
        while (Time.time - start < total)
        {
            float t = Time.time - start;
            while (seg < P - 1 && t > times[seg]) seg++;

            var a = traj.points[seg - 1];
            var b = traj.points[seg];
            float dt = Mathf.Max(times[seg] - times[seg - 1], 1e-4f);
            float u = Mathf.Clamp01((t - times[seg - 1]) / dt), u2 = u * u, u3 = u2 * u;
            float h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;

            for (int i = 0; i < n; i++)
            {
                if (bodies[i] == null) continue;
                float p0 = (float)a.positions[i], p1 = (float)b.positions[i];
                float v = hasVel
                    ? h00 * p0 + h10 * dt * (float)a.velocities[i] / scale + h01 * p1 + h11 * dt * (float)b.velocities[i] / scale
                    : Mathf.Lerp(p0, p1, u);
                SetTarget(bodies[i], v);
            }
            yield return null;
        }
        SetTargets(bodies, traj.points[P - 1].positions); // land exactly on the final point
    }

    static void SetTarget(ArticulationBody ab, double rad)
    {
        var d = ab.xDrive;
        d.target = (float)rad * Mathf.Rad2Deg;
        ab.xDrive = d;
    }

    static void SetTargets(ArticulationBody[] bodies, double[] pos)
    {
        for (int i = 0; i < bodies.Length && i < pos.Length; i++)
            if (bodies[i] != null) SetTarget(bodies[i], pos[i]);
    }

    // ---------------------------------------------------------------- settling

    bool IsSettled(out string reason)
    {
        foreach (var kv in joints)
        {
            var ab = kv.Value;
            if (ab.dofCount == 0) continue;
            float err = Mathf.Abs(ab.jointPosition[0] * Mathf.Rad2Deg - ab.xDrive.target);
            if (err > settleToleranceDeg) { reason = $"{kv.Key} pos error {err:F2} deg"; return false; }
            if (Mathf.Abs(ab.jointVelocity[0]) > settleVelocity) { reason = $"{kv.Key} still moving"; return false; }
        }
        reason = "OK";
        return true;
    }

    public IEnumerator WaitSettled()
    {
        float start = Time.time, stableSince = -1f;
        string reason = "";
        while (Time.time - start < settleTimeout)
        {
            if (IsSettled(out reason))
            {
                if (stableSince < 0) stableSince = Time.time;
                if (Time.time - stableSince >= 0.15f) yield break;
            }
            else stableSince = -1f;
            yield return null;
        }
        Debug.LogWarning($"[Arm] Settle timeout ({settleTimeout}s): {reason}");
    }
}