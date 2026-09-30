using System.Collections.Generic;
using UnityEngine;

public class ContactLogger : MonoBehaviour
{
    readonly Dictionary<Transform, float> lastTouch = new Dictionary<Transform, float>();
    readonly HashSet<string> touching = new HashSet<string>();

    void OnCollisionEnter(Collision c)
    {
        Debug.Log($"[Contact] {name} hit '{c.collider.name}' (parent '{c.collider.transform.parent?.name}') " +
                  $"relVel={c.relativeVelocity.magnitude:F2} contacts={c.contactCount} " +
                  $"firstPoint.y={c.GetContact(0).point.y:F3}");
    }

    void OnCollisionStay(Collision c)
    {
        lastTouch[c.collider.transform] = Time.time;
        var body = c.collider.attachedArticulationBody;
        touching.Add(body != null ? $"{body.transform.parent?.name}/{body.name}" : c.collider.name);
    }

    /// True if any collider under `link` touched this object within `window` seconds.
    public bool TouchedRecently(Transform link, float window = 0.15f)
    {
        if (link == null) return false;
        foreach (var kv in lastTouch)
            if (kv.Key != null && kv.Key.IsChildOf(link) && Time.time - kv.Value < window) return true;
        return false;
    }

    public string Summary()
    {
        string s = string.Join(",", touching);
        touching.Clear();
        return s;
    }
}