using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Ur10eRg2Moveit;  // CollisionObjectMsg
using RosMessageTypes.Shape;           // SolidPrimitiveMsg
using RosMessageTypes.Std;             // HeaderMsg
using RosMessageTypes.Geometry;        // PoseMsg, PointMsg, QuaternionMsg

/// Sends every BoxCollider in `obstacles` (table, basket walls, ...) to MoveIt as a collision object,
/// so the planner routes around them and never drives the gripper into the table.
public class PlanningSceneSync : MonoBehaviour
{
    [Tooltip("Same object as HarvestPoseClient.robotBase.")]
    public Transform robotBase;
    [Tooltip("MoveIt planning frame, normally the robot's base_link.")]
    public string frameId = "base_link";
    public string topic = "/collision_object";
    [Tooltip("Extra size added to every box dimension (metres) so the planner keeps a safety buffer.")]
    public float margin = 0.02f;
    public List<BoxCollider> obstacles = new List<BoxCollider>();

    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Explicitly register with the ROS package name expected by MoveIt
        ros.RegisterPublisher(topic, "moveit_msgs/CollisionObject");

        Invoke(nameof(PublishAll), 1.0f); // Give the TCP connection a moment to establish
    }

    [ContextMenu("Publish Obstacles")]
    public void PublishAll()
    {
        if (ros == null) ros = ROSConnection.GetOrCreateInstance();

        if (robotBase == null)
        {
            Debug.LogError("[PlanningSceneSync] Cannot publish: robotBase is unassigned in the Inspector!");
            return;
        }

        int publishedCount = 0;
        foreach (var box in obstacles)
        {
            if (box != null)
            {
                Publish(box);
                publishedCount++;
            }
        }
        Debug.Log($"[PlanningSceneSync] Finished publishing {publishedCount} obstacle(s) to MoveIt.");
    }

    void Publish(BoxCollider box)
    {
        Transform t = box.transform;
        Vector3 worldCenter = t.TransformPoint(box.center);
        Vector3 size = Vector3.Scale(box.size, t.lossyScale);
        Quaternion worldRot = t.rotation;

        // Unity (x right, y up, z forward) -> ROS (x forward, y left, z up)
        Vector3 lp = robotBase.InverseTransformPoint(worldCenter);
        Quaternion lr = Quaternion.Inverse(robotBase.rotation) * worldRot;

        var pose = new PoseMsg(
            new PointMsg(lp.z, -lp.x, lp.y),
            new QuaternionMsg(-lr.z, lr.x, -lr.y, lr.w)
        );

        var msg = new CollisionObjectMsg
        {
            header = new HeaderMsg { frame_id = frameId },
            id = box.gameObject.name,
            primitives = new[]
            {
                new SolidPrimitiveMsg
                {
                    type = SolidPrimitiveMsg.BOX,
                    // box axes swap: ROS x = Unity z, y = Unity x, z = Unity y (+ safety margin)
                    dimensions = new double[]
                    {
                        Mathf.Abs(size.z) + margin,
                        Mathf.Abs(size.x) + margin,
                        Mathf.Abs(size.y) + margin
                    }
                }
            },
            primitive_poses = new[] { pose },
            operation = CollisionObjectMsg.ADD
        };

        ros.Publish(topic, msg);
        Debug.Log($"[PlanningScene] Published '{msg.id}' to '{topic}'.");
    }
}
