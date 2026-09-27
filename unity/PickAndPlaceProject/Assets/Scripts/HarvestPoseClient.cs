using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Ur10eRg2Moveit;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

public class HarvestPoseClient : MonoBehaviour
{
    ROSConnection ros;
    const string serviceName = "harvest_pose";

    [Header("Scene references")]
    public Transform tomatoTarget;
    public Transform placementTarget;

    [Header("Arm joints, in this exact order")]
    public ArticulationBody shoulderPan;
    public ArticulationBody shoulderLift;
    public ArticulationBody elbow;
    public ArticulationBody wrist1;
    public ArticulationBody wrist2;
    public ArticulationBody wrist3;

    [Header("Gripper values")]
    public float gripperOpen = 0.0f;
    public float gripperClosed = 0.6f;

    bool sequenceRunning = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<HarvestPoseRequest, HarvestPoseResponse>(serviceName);
    }

    public void OnHarvestButtonClicked()
    {
        if (sequenceRunning)
        {
            Debug.LogWarning("Harvest sequence already running, ignoring click.");
            return;
        }
        StartCoroutine(RunHarvestSequence());
    }

    IEnumerator RunHarvestSequence()
    {
        sequenceRunning = true;

        if (tomatoTarget == null || placementTarget == null)
        {
            Debug.LogError("Tomato target or placement target not assigned.");
            sequenceRunning = false;
            yield break;
        }

        yield return SendAndAnimate(tomatoTarget.position, Quaternion.identity, gripperOpen, "Move to tomato");
        yield return SendAndAnimate(tomatoTarget.position, Quaternion.identity, gripperClosed, "Grasp tomato");
        yield return SendAndAnimate(placementTarget.position, Quaternion.identity, gripperClosed, "Move to basket");
        yield return SendAndAnimate(placementTarget.position, Quaternion.identity, gripperOpen, "Release tomato");

        Debug.Log("Harvest sequence complete.");
        sequenceRunning = false;
    }

    IEnumerator SendAndAnimate(Vector3 targetPosition, Quaternion targetRotation, float gripperPosition, string stepLabel)
    {
        bool done = false;
        HarvestPoseResponse result = null;

        HarvestPoseRequest request = new HarvestPoseRequest();
        request.target_pose = new PoseMsg(
            new PointMsg(targetPosition.z, -targetPosition.x, targetPosition.y),
            new QuaternionMsg(-targetRotation.z, targetRotation.x, -targetRotation.y, targetRotation.w)
        );
        request.gripper_position = gripperPosition;

        Debug.Log($"[{stepLabel}] Sending request...");

        ros.SendServiceMessage<HarvestPoseResponse>(serviceName, request, (HarvestPoseResponse response) =>
        {
            result = response;
            done = true;
        });

        yield return new WaitUntil(() => done);

        if (result == null || !result.success)
        {
            Debug.LogError($"[{stepLabel}] FAILED: {result?.message}");
            yield break;
        }

        Debug.Log($"[{stepLabel}] Plan OK, animating {result.trajectory.points.Length} points...");
        yield return PlayTrajectory(result.trajectory);
        Debug.Log($"[{stepLabel}] Animation complete.");
    }

    IEnumerator PlayTrajectory(JointTrajectoryMsg trajectory)
    {
        ArticulationBody[] joints = { shoulderPan, shoulderLift, elbow, wrist1, wrist2, wrist3 };

        // Map trajectory joint_names to our ordered joints array
        int[] jointIndexMap = new int[trajectory.joint_names.Length];
        string[] ourNames = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };

        for (int i = 0; i < trajectory.joint_names.Length; i++)
        {
            jointIndexMap[i] = System.Array.IndexOf(ourNames, trajectory.joint_names[i]);
        }

        double previousTime = 0;

        foreach (var point in trajectory.points)
        {
            double pointTime = point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
            float waitTime = (float)(pointTime - previousTime);
            previousTime = pointTime;

            for (int i = 0; i < point.positions.Length; i++)
            {
                int jointIdx = jointIndexMap[i];
                if (jointIdx < 0 || joints[jointIdx] == null) continue;

                var drive = joints[jointIdx].xDrive;
                drive.target = (float)(point.positions[i] * Mathf.Rad2Deg); // ArticulationBody uses degrees for angular drives
                joints[jointIdx].xDrive = drive;
            }

            if (waitTime > 0)
                yield return new WaitForSeconds(waitTime);
        }
    }
}