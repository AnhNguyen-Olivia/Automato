# ROS2 Humble + Unity Integration — Step-by-Step Guide

This walks through the `ros_unity_integration` tutorial series (Setup → Publisher → Subscriber → UnityService → ServiceCall) using **ROS2 Humble installed natively on Ubuntu 22.04** — no Docker needed, since Humble is the officially matched ROS2 release for jammy.

---

## Part 0: Install ROS2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add the ROS2 apt repository
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools python3-colcon-common-extensions

# Source it in every terminal automatically
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify it worked:
```bash
ros2 topic list
```
You should see `/parameter_events` and `/rosout`.

---

## Part 1: Set up the Colcon workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**Get the ROS2 branch of ROS-TCP-Endpoint:**
```bash
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

**Copy in the demo packages** from your Unity-Robotics-Hub clone (adjust the path to wherever you cloned it):
```bash
cp -r /PATH/TO/Unity-Robotics-Hub/tutorials/ros_unity_integration/ros2_packages/unity_robotics_demo .
cp -r /PATH/TO/Unity-Robotics-Hub/tutorials/ros_unity_integration/ros2_packages/unity_robotics_demo_msgs .
```

Example
```
cp -r /home/olivia/Unity-Robotics-Hub/tutorials/ros_unity_integration/ros2_packages/unity_robotics_demo .
cp -r /home/olivia/Unity-Robotics-Hub/tutorials/ros_unity_integration/ros2_packages/unity_robotics_demo_msgs .
```

Your `src` folder should now contain three packages: `ROS-TCP-Endpoint`, `unity_robotics_demo`, `unity_robotics_demo_msgs`.

**Build (note: you source twice, on purpose):**
```bash
cd ~/ros2_ws
source install/setup.bash    # only does anything after the first build, harmless before that
colcon build
source install/setup.bash
```

Check for errors — a clean build prints `Summary: 3 packages finished`.

---

## Part 2: Start the TCP endpoint

Open a terminal, and each time you open a new terminal for this project run:
```bash
cd ~/ros2_ws
source install/setup.bash
```

Find your machine's IP (Unity needs this to connect):
```bash
hostname -I
```
Take the first address printed (e.g. `192.168.1.42`).

Start the endpoint:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your IP address>
```
Replace `<your IP address>` with the address from `hostname -I`. You'll see something like:
```
[INFO] [...]: Starting server on 192.168.1.42:10000
```
Leave this terminal running for the rest of the tutorial.

> If you'd rather use a fixed port, add `-p ROS_TCP_PORT:=10000` to the same command (10000 is already the default).

---

## Part 3: Unity-side setup

1. Open Unity Hub, create/open a project (Unity 2020 or newer).
2. **Window → Package Manager → `+` → Add package from git URL**, paste:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```
3. Open **Robotics → ROS Settings** from the Unity menu bar.
   - Set **ROS IP Address** to the same IP you used above.
   - Switch **Protocol** to **ROS2**.
4. **Robotics → Generate ROS Messages...** In the Message Browser, click **Browse**, and point it at:
   ```
   tutorials/ros_unity_integration/ros_packages/unity_robotics_demo_msgs
   ```
   (the doc notes the ROS1 and ROS2 versions of this msgs folder generate identical C# — either works, no need to regenerate when switching).
5. Expand `unity_robotics_demo_msgs`, click **Build 2 msgs** and **Build 2 srvs**. Files land in `Assets/RosMessages/UnityRoboticsDemo/msg` and `.../srv`.

---

## Part 4: Publisher (Unity → ROS2)

**Create `RosPublisherExample.cs`** in Unity, attach this code:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;

public class RosPublisherExample : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "pos_rot";
    public GameObject cube;
    public float publishMessageFrequency = 0.5f;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(topicName);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)
        {
            cube.transform.rotation = Random.rotation;
            PosRotMsg cubePos = new PosRotMsg(
                cube.transform.position.x, cube.transform.position.y, cube.transform.position.z,
                cube.transform.rotation.x, cube.transform.rotation.y, cube.transform.rotation.z, cube.transform.rotation.w
            );
            ros.Publish(topicName, cubePos);
            timeElapsed = 0;
        }
    }
}
```

**Scene setup:**
- Add a Plane and a Cube (Hierarchy → `+`), lift the cube above the plane.
- Create an empty GameObject named `RosPublisher`, attach the script, drag the Cube into the `Cube` field.
- Press Play. The connection light top-left of the Game view should turn blue.

**Verify from ROS2** (new terminal):
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 topic echo pos_rot
```
You should see position/rotation values streaming in every 0.5s.

---

## Part 5: Subscriber (ROS2 → Unity)

**Create `RosSubscriberExample.cs`:**

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosColor = RosMessageTypes.UnityRoboticsDemo.UnityColorMsg;

public class RosSubscriberExample : MonoBehaviour
{
    public GameObject cube;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<RosColor>("color", ColorChange);
    }

    void ColorChange(RosColor colorMessage)
    {
        cube.GetComponent<Renderer>().material.color =
            new Color32((byte)colorMessage.r, (byte)colorMessage.g, (byte)colorMessage.b, (byte)colorMessage.a);
    }
}
```

- Create an empty GameObject `RosSubscriber`, attach the script, drag the Cube into `cube`.
- Press Play.

**Trigger a color change from ROS2:**
```bash
ros2 run unity_robotics_demo color_publisher
```
The cube in Unity should change to a random color.

---

## Part 6: UnityService (ROS2 calls into Unity)

**Create `RosUnityServiceExample.cs`:**

```csharp
using RosMessageTypes.UnityRoboticsDemo;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class RosUnityServiceExample : MonoBehaviour
{
    [SerializeField] string m_ServiceName = "obj_pose_srv";

    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .ImplementService<ObjectPoseServiceRequest, ObjectPoseServiceResponse>(m_ServiceName, GetObjectPose);
    }

    private ObjectPoseServiceResponse GetObjectPose(ObjectPoseServiceRequest request)
    {
        Debug.Log("Received request for object: " + request.object_name);
        ObjectPoseServiceResponse response = new ObjectPoseServiceResponse();
        GameObject obj = GameObject.Find(request.object_name);
        if (obj)
        {
            response.object_pose.position = obj.transform.position.To<FLU>();
            response.object_pose.orientation = obj.transform.rotation.To<FLU>();
        }
        return response;
    }
}
```

- Create empty GameObject `UnityService`, attach the script. Press Play.

**Call it from ROS2:**
```bash
ros2 service call obj_pose_srv unity_robotics_demo_msgs/ObjectPoseService "{object_name: Cube}"
```
Expected output includes the cube's position/orientation, e.g.:
```
response:
unity_robotics_demo_msgs.srv.ObjectPoseService_Response(object_pose=geometry_msgs.msg.Pose(position=..., orientation=...))
```

---

## Part 7: Service Call (Unity calls into ROS2)

**Start the ROS2 position service** (new terminal, sourced as usual):
```bash
ros2 run unity_robotics_demo position_service
```

**Create `RosServiceCallExample.cs`:**

```csharp
using RosMessageTypes.UnityRoboticsDemo;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class RosServiceCallExample : MonoBehaviour
{
    ROSConnection ros;
    public string serviceName = "pos_srv";
    public GameObject cube;
    public float delta = 1.0f;
    public float speed = 2.0f;
    private Vector3 destination;
    float awaitingResponseUntilTimestamp = -1;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<PositionServiceRequest, PositionServiceResponse>(serviceName);
        destination = cube.transform.position;
    }

    private void Update()
    {
        float step = speed * Time.deltaTime;
        cube.transform.position = Vector3.MoveTowards(cube.transform.position, destination, step);

        if (Vector3.Distance(cube.transform.position, destination) < delta && Time.time > awaitingResponseUntilTimestamp)
        {
            PosRotMsg cubePos = new PosRotMsg(
                cube.transform.position.x, cube.transform.position.y, cube.transform.position.z,
                cube.transform.rotation.x, cube.transform.rotation.y, cube.transform.rotation.z, cube.transform.rotation.w
            );
            PositionServiceRequest req = new PositionServiceRequest(cubePos);
            ros.SendServiceMessage<PositionServiceResponse>(serviceName, req, Callback_Destination);
            awaitingResponseUntilTimestamp = Time.time + 1.0f;
        }
    }

    void Callback_Destination(PositionServiceResponse response)
    {
        awaitingResponseUntilTimestamp = -1;
        destination = new Vector3(response.output.pos_x, response.output.pos_y, response.output.pos_z);
        Debug.Log("New Destination: " + destination);
    }
}
```

- Create empty GameObject `RosService`, attach the script, drag the Cube into `cube`.
- Press Play — the cube should start moving to random positions, driven by responses from the ROS2 `position_service` node.

---

## Common issues

| Symptom | Fix |
|---|---|
| `Failed to resolve message name: No module named unity_robotics_demo_msgs.msg` | You skipped or forgot to `colcon build` + re-source after adding the demo packages. |
| Connection light stays red/grey in Unity | Double-check the IP in **Robotics → ROS Settings** matches `hostname -I` output, and that the endpoint terminal is still running. |
| `ros2 topic echo` shows nothing | Confirm Play mode is active in Unity and the publisher script is actually attached and referencing the cube. |
| Protocol mismatch errors | Make sure ROS Settings in Unity is set to **ROS2**, not ROS1 — this is easy to forget if you followed a ROS1 guide first. |

---

## Where this leaves you for the course project

This gives you the **communication layer** (Unity ↔ ROS2 Humble) working natively on your 22.04 dual-boot. Your obstacle-avoidance project still needs the planning side — MoveIt2 for whichever robot you pick (UR10e, TurtleBot3, YouBot, or M200V2), plus dynamic collision-object updates in the planning scene. That's a separate build on top of this foundation, not covered by the `ros_unity_integration` tutorial itself.
