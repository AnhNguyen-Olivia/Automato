using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// Owns the RG2 gripper joints. Commands are in radians of the lead knuckle; mimic joints follow via multipliers.
public class GripperController : MonoBehaviour
{
    public ArmController arm;

    [System.Serializable]
    public class JointSetting
    {
        public string linkName;
        public float multiplier = 1f;
    }

    [Header("Command values (rad of the lead knuckle)")]
    public float open = 0f;
    public float closed = 0.6f;
    public string leadLink = "left_outer_knuckle";

    [Header("Drive")]
    public float stiffness = 20000f;
    public float damping = 2000f;
    public float forceLimit = 100f;

    [Header("Settling")]
    public float settleVelocity = 0.06f;
    public float settleTimeout = 1.5f;

    [Tooltip("Verify each multiplier against the <mimic> tags in the gripper URDF.")]
    public List<JointSetting> joints = new List<JointSetting>
    {
        new JointSetting { linkName = "left_outer_knuckle",  multiplier =  1f },
        new JointSetting { linkName = "left_inner_knuckle",  multiplier =  1f },
        new JointSetting { linkName = "left_inner_finger",   multiplier = -1f },
        new JointSetting { linkName = "right_outer_knuckle", multiplier =  1f },
        new JointSetting { linkName = "right_inner_knuckle", multiplier =  1f },
        new JointSetting { linkName = "right_inner_finger",  multiplier = -1f },
    };

    readonly Dictionary<string, ArticulationBody> bodies = new Dictionary<string, ArticulationBody>();

    /// Lead knuckle angle in rad (NaN if not found).
    public float LeadPosition =>
        bodies.TryGetValue(leadLink, out var ab) && ab.dofCount > 0 ? ab.jointPosition[0] : float.NaN;

    void Awake() { Find(); }

    [ContextMenu("Find Gripper Joints")]
    public void Find()
    {
        bodies.Clear();
        if (arm == null || arm.Root == null) { Debug.LogError("[Gripper] Assign arm (with robotBase set)."); return; }
        foreach (var ab in arm.Root.GetComponentsInChildren<ArticulationBody>())
            foreach (var j in joints)
                if (ab.name == j.linkName) bodies[j.linkName] = ab;
        foreach (var j in joints)
            if (!bodies.ContainsKey(j.linkName)) Debug.LogWarning($"[Gripper] Link '{j.linkName}' not found.");
    }

    public void Set(float rad)
    {
        if (bodies.Count == 0) Find();
        foreach (var j in joints)
        {
            if (!bodies.TryGetValue(j.linkName, out var ab)) continue;
            var d = ab.xDrive;
            d.stiffness = stiffness; d.damping = damping; d.forceLimit = forceLimit;
            d.target = rad * j.multiplier * Mathf.Rad2Deg;
            ab.xDrive = d;
        }
    }

    public IEnumerator WaitSettled()
    {
        yield return new WaitForSeconds(0.2f);
        float start = Time.time, stableSince = -1f;
        while (Time.time - start < settleTimeout)
        {
            bool still = true;
            foreach (var ab in bodies.Values)
                if (ab.dofCount > 0 && Mathf.Abs(ab.jointVelocity[0]) > settleVelocity) { still = false; break; }

            if (still)
            {
                if (stableSince < 0) stableSince = Time.time;
                if (Time.time - stableSince >= 0.15f) yield break;
            }
            else stableSince = -1f;
            yield return null;
        }
    }

    /// True if the lead knuckle closed some way but stopped before fully closed, i.e. something is between the fingers.
    public bool HoldsObject()
    {
        float pos = LeadPosition;
        if (float.IsNaN(pos)) return true; // cannot tell; do not block the sequence
        return Mathf.Abs(pos - open) > 0.05f && Mathf.Abs(closed - pos) > 0.05f;
    }

    [ContextMenu("Test Gripper")]
    void Test() { StartCoroutine(TestRoutine()); }

    IEnumerator TestRoutine()
    {
        Set(open);   yield return new WaitForSeconds(1f);
        Debug.Log($"[Gripper] open   lead={LeadPosition:F2}");
        Set(closed); yield return new WaitForSeconds(1.5f);
        Debug.Log($"[Gripper] closed lead={LeadPosition:F2} holding={HoldsObject()}");
        Set(open);
    }
}