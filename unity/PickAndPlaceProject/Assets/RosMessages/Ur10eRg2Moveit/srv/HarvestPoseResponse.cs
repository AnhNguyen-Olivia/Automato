using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ur10eRg2Moveit
{
    [Serializable]
    public class HarvestPoseResponse : Message
    {
        public const string k_RosMessageName = "ur10e_rg2_moveit/HarvestPose";
        public override string RosMessageName => k_RosMessageName;

        public bool success;
        public string message;
        public RosMessageTypes.Trajectory.JointTrajectoryMsg trajectory;

        public HarvestPoseResponse()
        {
            this.success = false;
            this.message = "";
            this.trajectory = new RosMessageTypes.Trajectory.JointTrajectoryMsg();
        }

        public HarvestPoseResponse(bool success, string message, RosMessageTypes.Trajectory.JointTrajectoryMsg trajectory)
        {
            this.success = success;
            this.message = message;
            this.trajectory = trajectory;
        }

        public static HarvestPoseResponse Deserialize(MessageDeserializer deserializer) => new HarvestPoseResponse(deserializer);

        private HarvestPoseResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.success);
            deserializer.Read(out this.message);
            this.trajectory = RosMessageTypes.Trajectory.JointTrajectoryMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.success);
            serializer.Write(this.message);
            serializer.Write(this.trajectory);
        }

        public override string ToString()
        {
            return "HarvestPoseResponse: " +
            "\nsuccess: " + success.ToString() +
            "\nmessage: " + message.ToString() +
            "\ntrajectory: " + trajectory.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
