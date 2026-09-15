# Installing MoveIt2 on ROS2 Humble (Ubuntu 22.04)

MoveIt2 has two install paths. Pick based on what you need:

- **Binary install** — fast (~5 min), gives you the MoveIt2 libraries/packages to build your own robot's planning setup. Good if you're writing your own MoveIt2 config for TurtleBot3/UR10e/YouBot/M200V2 from scratch.
- **Source + tutorials workspace** — slower (~20-30 min build), but gives you the official `moveit2_tutorials` demo (Panda arm) plus the full source tree, which is the better starting point for learning the APIs before adapting to your own robot.

For your course project, I'd recommend doing **both**: source+tutorials first to get a working reference demo, then binary-equivalent packages as you build your own robot's config. The steps below assume ROS2 Humble is already installed (from the earlier guide).

---

## Part 0: Prerequisites check

```bash
# Confirm you're on 22.04
lsb_release -a

# Confirm Humble is installed and sourced
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
# should print: humble
```

---

## Part 1 (recommended first step): Source install with the tutorials workspace

This is the official MoveIt2 "Getting Started" path — gives you a working demo and the full source so you can read/modify planning code.

### 1. Install rosdep and update system
```bash
sudo apt install python3-rosdep
sudo rosdep init      # skip this line if you already ran rosdep init before
rosdep update
sudo apt update
sudo apt dist-upgrade
```

### 2. Create the workspace and get the source
```bash
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src
git clone --branch humble https://github.com/ros-planning/moveit2_tutorials
vcs import < moveit2_tutorials/moveit2_tutorials.repos
```
> `vcs import` may prompt for GitHub credentials — just press Enter through it; an "Authentication failed" message here is expected and harmless (it's only trying public repos).

### 3. Install dependencies
```bash
cd ~/ws_moveit2
sudo apt update
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

### 4. Build
```bash
cd ~/ws_moveit2
colcon build --mixin release --executor sequential
```
- `--executor sequential` builds one package at a time — slower, but avoids your machine running out of RAM/freezing on the heavier packages (recommended unless you have 32GB+ RAM).
- This step genuinely takes 20-30 minutes. Don't panic if it looks stuck — check `htop` in another terminal if you're worried.
- If you have plenty of RAM and want it faster, drop `--executor sequential`.

### 5. Source it automatically
```bash
echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 6. Switch to CycloneDDS (recommended — fixes a known default-RMW issue)
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

### 7. Run the demo
```bash
ros2 launch moveit2_tutorials demo.launch.py
```
RViz should open with the Panda arm, an interactive marker, and a "Plan & Execute" panel. If it launches without errors, MoveIt2 is working.

> **Known gotcha:** if `demo.launch.py` errors out about missing packages, re-run the `vcs import` step — an incomplete import is a common cause (tracked in moveit2_tutorials issue #824). Re-running `vcs import src < src/moveit2_tutorials/moveit2_tutorials.repos` from `~/ws_moveit2` and rebuilding fixes it.

---

## Part 2: Binary install (for building your own robot config)

Once you've confirmed the tutorial demo works, install the binary MoveIt2 packages into your **project** workspace (e.g. the `~/ros2_ws` from the Unity integration guide) so you can build your robot-specific MoveIt config alongside it.

```bash
sudo apt update
sudo apt install ros-humble-moveit
```

This pulls in the full MoveIt2 package set (`moveit_core`, `moveit_ros_planning`, `moveit_ros_move_group`, `moveit_planners_ompl`, `moveit_setup_assistant`, etc.) as Debian packages — no build required.

> **Don't mix source and binary MoveIt2 in the same workspace.** If you built from source in `~/ws_moveit2`, keep your own robot's packages there too, or in a workspace that doesn't also have `ros-humble-moveit` installed — ROS2 will get confused about which MoveIt libraries to use if both are visible at once.

---

## Part 3: Generate your robot's MoveIt config

For your project, once you've picked a robot (UR10e / TurtleBot3 / YouBot / M200V2), you'll need a MoveIt2 config package for it — SRDF, kinematics, controllers, planning config.

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```
This launches the graphical Setup Assistant. Point it at your robot's URDF (from the Unity-Robotics-Hub URDF importer or your robot's official ROS2 description package) and it walks you through generating the config package.

> UR10e already has an actively maintained ROS2 driver/MoveIt config (`Universal_Robots_ROS2_Driver` + `ur_moveit_config`) — if you pick that robot, check whether you can use the existing config as a starting point rather than building from scratch, which saves significant time for your obstacle-avoidance work.

---

## Common issues

| Symptom | Fix |
|---|---|
| `colcon build` freezes or the machine becomes unresponsive | Re-run with `--executor sequential`, and close other heavy apps (especially if on a VM — recall this really should be native/dual-boot, not a VM). |
| `demo.launch.py` fails with missing package errors | Re-run `vcs import` from `~/ws_moveit2` and rebuild — a partial import is the most common cause. |
| RMW/DDS discovery errors or nodes not seeing each other | Install and export `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` as in step 6 above. |
| Conflicts between source-built and apt-installed MoveIt | Keep them in separate workspaces; don't source both `setup.bash` files in the same terminal session. |

---

## Where this leaves you

You now have:
1. A working MoveIt2 reference demo (Panda arm) to learn from.
2. Binary MoveIt2 packages ready for your own robot's config.
3. A path to generate that config via Setup Assistant.

Next step for the obstacle-avoidance requirement specifically: MoveIt2's **planning scene** API (`moveit_msgs/CollisionObject`, `PlanningSceneInterface`) is what you'll use to add/update/remove obstacles dynamically at runtime, and the **Hybrid Planning** feature (new in Humble) is worth looking at for replanning around obstacles that move during execution rather than only at planning time. Want a guide for wiring that up next?
