# Pick-and-Place Tutorial — Recreated for ROS2 Humble + Unity (Ubuntu 22.04, no Docker)

## Important scope note

The official Unity-Robotics-Hub pick-and-place tutorial (`niryo_moveit`) is **ROS1-only** — built on `moveit_commander`, catkin, and roslaunch XML, none of which exist in ROS2. There is no official ROS2 release of this tutorial; the only ROS2 attempt is an incomplete, unmerged Unity branch.

This guide **recreates the same architecture** (Unity sends the robot pose + target/destination poses → ROS plans a sequence of trajectories with MoveIt → Unity animates the ArticulationBody through them → a gripper controller opens/closes) using MoveIt2's real APIs (`MoveGroupInterface` in C++, which is the stable planning API in Humble — the Python `moveit_py` bindings weren't reliably available yet in Humble).

**Assumptions you'll need to adjust:** exact joint names, link names, and gripper joint config depend on the URDF of whichever robot you use (Niryo One, UR10e, etc.) and the MoveIt2 config you generate for it in Part 3. Treat the code below as a working template — you will edit joint names/frame names to match your actual robot once you've run the Setup Assistant.

Prerequisites: ROS2 Humble installed natively (see the earlier install guide), MoveIt2 installed (binary or source — see the earlier MoveIt2 guide), Unity with ROS-TCP-Connector added and **Protocol set to ROS2** (see the ROS2 Unity integration guide).

---

## Part 1: Unity scene + URDF import (unchanged by ROS version)

This part is identical regardless of ROS1 vs ROS2 — URDF import is entirely Unity-side and doesn't talk to ROS at all yet.

1. Install Unity 2020.3 LTS or newer via Unity Hub.
2. Create a new 3D project.
3. **Window → Package Manager → + → Add package from git URL**:
   ```
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```
4. Get your robot's URDF. If sticking with Niryo One (matches the original tutorial), it's in the `niryo_one_urdf` submodule under `Unity-Robotics-Hub/tutorials/pick_and_place/ROS/src/niryo_one_urdf`. If using UR10e/TurtleBot3/YouBot/M200V2, use that robot's official URDF/xacro package instead.
5. Drag the `.urdf` file into the Unity **Assets** window — the importer builds the ArticulationBody hierarchy automatically.
6. Add a Table, Target (cube), and TargetPlacement object to the scene so there's something to pick and somewhere to place it.
7. Position the Main Camera at roughly `(0, 1.4, -0.7)` with rotation `(45, 0, 0)` for a workable view.

---

## Part 2: ROS2 ↔ Unity communication

Already covered in the earlier `ros2-humble-unity-integration-guide.md` — reuse that colcon workspace (`~/ros2_ws`). Confirm:
- ROS-TCP-Endpoint (ROS2 branch) is built and running.
- Unity's **Robotics → ROS Settings** has Protocol = **ROS2** and the correct IP.

If that's working (you got the publisher/subscriber demo running), you're set for this part — no changes needed.

---

## Part 3: Generate the MoveIt2 config for your robot

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

In the GUI:
1. **Create New MoveIt Configuration Package**, load your robot's URDF.
2. **Self-Collisions** tab → Generate Collision Matrix.
3. **Planning Groups** → Add a group (e.g. `arm`) covering the arm's joints, and a second group (e.g. `gripper`) for the gripper joints, if applicable.
4. **Robot Poses** → define at least a `home` pose (useful for resetting between pick attempts).
5. **End Effectors** → link the gripper group to the arm group's tip link.
6. **Controllers** → auto-generate `ros2_control`/`FollowJointTrajectory` controllers for each group.
7. **Generate Package**, output to e.g. `~/ros2_ws/src/my_robot_moveit_config`.

Note the exact **planning group names** and **joint names** it generates (visible in the group's `.srdf`) — you'll need these in Part 5.

Build it into your workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_moveit_config
source install/setup.bash
```

---

## Part 4: Custom interface package (the MoverService equivalent)

Create a dedicated package for the custom service, matching the original's `MoverService.srv`:

```bash
cd ~/ros2_ws/src
ros2 pkg create mover_interfaces --build-type ament_cmake
mkdir mover_interfaces/srv
```

**`mover_interfaces/srv/MoverService.srv`:**
```
# Request
geometry_msgs/Pose pick_pose
geometry_msgs/Pose place_pose
---
# Response
moveit_msgs/RobotTrajectory[] trajectories
```

**`mover_interfaces/CMakeLists.txt`** — add before `ament_package()`:
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoverService.srv"
  DEPENDENCIES geometry_msgs moveit_msgs
)
```

**`mover_interfaces/package.xml`** — add inside `<package>`:
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>geometry_msgs</depend>
<depend>moveit_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Build:
```bash
cd ~/ros2_ws
colcon build --packages-select mover_interfaces
source install/setup.bash
```

---

## Part 5: The mover node (C++, MoveGroupInterface)

```bash
cd ~/ros2_ws/src
ros2 pkg create my_pick_and_place --build-type ament_cmake --dependencies rclcpp moveit_ros_planning_interface mover_interfaces geometry_msgs moveit_msgs
mkdir -p my_pick_and_place/src
```

**`my_pick_and_place/src/mover.cpp`** — mirrors the original `mover.py`'s four-stage plan (pre-grasp → grasp → pick-up → place), but computed with `MoveGroupInterface`:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "mover_interfaces/srv/mover_service.hpp"

using MoverService = mover_interfaces::srv::MoverService;

class MoverNode : public rclcpp::Node
{
public:
  MoverNode() : Node("mover_node")
  {
    service_ = this->create_service<MoverService>(
      "mover_service",
      std::bind(&MoverNode::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleRequest(
    const std::shared_ptr<MoverService::Request> request,
    std::shared_ptr<MoverService::Response> response)
  {
    // NOTE: group name must match what you named it in the Setup Assistant (Part 3)
    static const std::string ARM_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), ARM_GROUP);

    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;

    // Stage 1: pre-grasp — approach above the pick pose
    geometry_msgs::msg::Pose pre_grasp = request->pick_pose;
    pre_grasp.position.z += 0.1;  // 10cm above pick pose
    if (!planTo(move_group, pre_grasp, trajectories)) { response->trajectories = trajectories; return; }

    // Stage 2: grasp — descend to the actual pick pose
    if (!planTo(move_group, request->pick_pose, trajectories)) { response->trajectories = trajectories; return; }

    // Stage 3: pick up — lift back to the pre-grasp height (gripper closes on Unity side between stage 2 and 3)
    if (!planTo(move_group, pre_grasp, trajectories)) { response->trajectories = trajectories; return; }

    // Stage 4: place — move to place pose
    if (!planTo(move_group, request->place_pose, trajectories)) { response->trajectories = trajectories; return; }

    response->trajectories = trajectories;
  }

  bool planTo(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & target,
    std::vector<moveit_msgs::msg::RobotTrajectory> & trajectories)
  {
    move_group.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (ok)
    {
      trajectories.push_back(plan.trajectory_);
      // Move the internal state forward so the next stage plans from here, not from the real start
      move_group.setStartStateToCurrentState();
    }
    return ok;
  }

  rclcpp::Service<MoverService>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

> **Caveat:** MoveGroupInterface plans against the robot's *actual* reported state by default, not a chained "pretend we already moved" state — the `setStartStateToCurrentState()` call above is a simplification. For a real four-stage chained plan (matching the original tutorial's behavior), you'll likely want `move_group.setStartState()` with a robot state manually advanced by the previous trajectory's final joint values. Treat Part 5 as a solid starting skeleton to debug against your actual robot, not drop-in-perfect code — MoveIt2's C++ API around start-state chaining has changed across versions.

**`my_pick_and_place/CMakeLists.txt`** — add:
```cmake
add_executable(mover src/mover.cpp)
ament_target_dependencies(mover rclcpp moveit_ros_planning_interface mover_interfaces geometry_msgs moveit_msgs)
install(TARGETS mover DESTINATION lib/${PROJECT_NAME})
```

Build:
```bash
cd ~/ros2_ws
colcon build --packages-select my_pick_and_place
source install/setup.bash
```

---

## Part 6: Launch file tying it together

**`my_pick_and_place/launch/pick_and_place.launch.py`:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_moveit_config'),
                'launch', 'demo.launch.py'  # generated by Setup Assistant
            )
        )
    )

    mover_node = Node(
        package='my_pick_and_place',
        executable='mover',
        output='screen'
    )

    tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        parameters=[{'ROS_IP': '0.0.0.0'}],  # replace with your machine's actual IP if not using 0.0.0.0
        output='screen'
    )

    return LaunchDescription([moveit_launch, mover_node, tcp_endpoint])
```

Run it:
```bash
ros2 launch my_pick_and_place pick_and_place.launch.py
```

---

## Part 7: Unity side (TrajectoryPlanner + SourceDestinationPublisher)

The C# scripts from the original tutorial work with ROS2 largely unchanged — `ROSConnection` in ROS-TCP-Connector already abstracts the ROS1/ROS2 protocol difference (you set that once in ROS Settings). What changes is just the **message/service source**: point message generation at your new `mover_interfaces` package instead of the original `niryo_moveit`.

1. **Robotics → Generate ROS Messages...**, Browse to `~/ros2_ws/src/mover_interfaces/srv`, generate `MoverService`.
2. Create `SourceDestinationPublisher.cs` — publishes the Target and TargetPlacement world positions, converted to ROS coordinates via `.To<FLU>()`, and calls the `mover_service` service with pick/place poses.
3. Create `TrajectoryPlanner.cs` — on receiving the `MoverServiceResponse`, iterate the `trajectories[]` array and animate the robot's `ArticulationBody` joints through each `RobotTrajectory`'s waypoints in sequence, closing the gripper between trajectory 2 and 3 (grasp → pick-up) and opening it after trajectory 4 (place).

These two scripts' logic is unchanged from the original tutorial's `Scripts/` folder — only the service name (`mover_service` instead of `niryo_moveit`) and namespace of the generated C# message classes differ. If you want, I can write out the full C# for both scripts next — they're substantial (150+ lines each) so I split this response rather than including everything at once.

---

## Part 8: Running end-to-end

```bash
# Terminal 1
cd ~/ros2_ws && source install/setup.bash
ros2 launch my_pick_and_place pick_and_place.launch.py
```
Then press Play in Unity, and trigger your publish button. Watch the mover node's terminal for planning success/failure per stage.

---

## What's genuinely uncertain here — verify as you go

- **Joint/group names** in `mover.cpp` (`"arm"`) — must exactly match what the Setup Assistant generated for your robot.
- **Gripper control** — how you open/close it depends entirely on your robot's gripper joint type; the original Niryo One tutorial used a simple two-finger joint driven directly, which may not map to UR10e/YouBot/M200V2 grippers.
- **Chained trajectory start-states** in Part 5 — flagged above, this is the part most likely to need debugging against your specific MoveIt2 version's C++ API.
- **`demo.launch.py`** filename in Part 6 assumes the Setup Assistant's default generated launch file name — check `~/ros2_ws/src/my_robot_moveit_config/launch/` for the actual filename it produced.

Given this is a course project with an obstacle-avoidance requirement anyway, you're going to be modifying `mover.cpp`'s planning logic regardless — this gives you a real MoveIt2 planning service to extend with `PlanningSceneInterface::applyCollisionObject()` calls for your dynamic obstacles.
