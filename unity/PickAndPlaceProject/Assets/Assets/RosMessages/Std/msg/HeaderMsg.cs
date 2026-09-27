using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Std
{
    [Serializable]
    public class HeaderMsg : Message
    {
        public const string k_RosMessageName = "std_msgs/Header";
        public override string RosMessageName => k_RosMessageName;

        public uint seq;
        public RosMessageTypes.BuiltinInterfaces.TimeMsg stamp;
        public string frame_id;

        public HeaderMsg()
        {
            this.seq = 0;
            this.stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg();
            this.frame_id = "";
        }

        public HeaderMsg(uint seq, RosMessageTypes.BuiltinInterfaces.TimeMsg stamp, string frame_id)
        {
            this.seq = seq;
            this.stamp = stamp;
            this.frame_id = frame_id;
        }

        public static HeaderMsg Deserialize(MessageDeserializer deserializer) => new HeaderMsg(deserializer);

        private HeaderMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.seq);
            this.stamp = RosMessageTypes.BuiltinInterfaces.TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.frame_id);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.seq);
            serializer.Write(this.stamp);
            serializer.Write(this.frame_id);
        }

        public override string ToString()
        {
            return "HeaderMsg: " +
            "\nseq: " + seq.ToString() +
            "\nstamp: " + stamp.ToString() +
            "\nframe_id: " + frame_id.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}