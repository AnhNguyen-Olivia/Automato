# Obstacle Avoidance Addendum — Tomato-Harvesting Robot Arm (Unity + ROS 1)

**Covers sections 15–28.** This extends the main guide (sections 1–14). Obstacle avoidance is a required subsystem, so everything here is written as a buildable, staged implementation, not theory.

---

## 0. Read this first

### 0.1 Assumptions about sections 1–14

I have not seen sections 1–14, so the names below are **placeholders**. Reconcile them with your existing project before copying any code.

| Placeholder | Meaning | Change to match |
|---|---|---|
| `ROS Noetic` + `MoveIt 1` | ROS 1 distribution and planner | Your ROS setup section |
| `base_link` | Robot base frame (also the planning frame) | Your URDF |
| `arm` | MoveIt planning group name | Your SRDF / MoveIt config |
| `/tomato_position` | `geometry_msgs/PoseStamped`, tomato target in `base_link` | Your tomato-localization node |
| `/joint_states` | `sensor_msgs/JointState` published from Unity | Your Unity robot publisher |
| `/camera/image_raw` | RGB image from Unity | Your camera section |

### 0.2 What is verified, what is my design, what you must check

**Verified against documentation while writing this:**

- Unity Robotics Hub supports **ROS 1 Noetic** (and Melodic) with ROS-TCP-Connector (Unity side) and ROS-TCP-Endpoint (ROS side). Its pick-and-place tutorial imports a URDF robot, calls a MoveIt planning service from Unity, and moves an articulated arm in Unity along the returned trajectory.
- `ROSConnection.GetOrCreateInstance()`, `RegisterPublisher<T>(topic)`, `Publish(topic, msg)` and `Subscribe<T>(topic, callback)` are the current Unity API. ROS-TCP-Connector also ships `ROSGeometry` (Unity ↔ ROS coordinate conversion, `To<FLU>()`).
- MoveIt 1's Noetic tutorials describe themselves as the latest and last MoveIt 1 version. `moveit_commander.PlanningSceneInterface` has `add_box`, `add_sphere`, `add_cylinder`, `add_object`, `attach_box`, `attach_object`, `remove_world_object`, `remove_attached_object`, `get_known_object_names`, `apply_planning_scene`, `clear`. It publishes collision objects on `/collision_object`.
- MoveIt's occupancy-map updaters accept `sensor_msgs/PointCloud2` (`PointCloudOctomapUpdater`) or depth `sensor_msgs/Image` (`DepthImageOctomapUpdater`). The documented example config uses `max_update_rate: 1.0`, so the octomap path is slow for fast-moving obstacles. Voxels overlapped by explicit collision objects are filtered out of the octomap.

**My design decisions (not from documentation):** the topic names under `/harvest/*`, the `harvest_msgs` custom message, the two-layer safety monitor, the state machine, the safety-margin scheme, and all thresholds.

**Unexecuted code:** the code in this addendum is reference code that I have **not run**. Treat it as a starting skeleton. Places where an API detail depends on your version are marked `VERIFY`.

### 0.3 A note on ROS 1

ROS Noetic reached end of life in 2025 and MoveIt 1 is in maintenance. For a university project that is fine (the Unity Hub tooling targets it and Docker avoids host-OS problems), but state it as a limitation in your report. Nothing in this design is ROS-1-specific except the Python node boilerplate and MoveIt 1 API names.

---

## 15. Obstacle Detection and Avoidance Architecture

### 15.1 The one idea that drives the whole design

> **Unity and MoveIt each have their own collision world, and they do not share it automatically.**

- Unity colliders decide what *physically* touches in the simulation.
- MoveIt's **planning scene** decides what the *planner believes* is free.

If you drop a box into Unity, the arm will still plan straight through it, because MoveIt has never heard of it. Therefore every obstacle must be **reported from Unity to ROS**, **converted into a planning-scene object**, and the planner must be re-run when the scene changes. The rest of this addendum is the machinery for that.

### 15.2 Pipeline

```
Camera / Sensors (Unity)
        ↓
Obstacle Detection            (ROS node: obstacle_detection)
        ↓
Obstacle Representation       (harvest_msgs/ObstacleArray: shape, pose, size, velocity)
        ↓
Static + Dynamic Obstacle Map (ROS node: scene_manager → MoveIt planning scene)
        ↓
Path Planning                 (MoveIt / OMPL, via harvest_fsm)
        ↓
Collision Checking            (MoveIt FCL checker + safety_monitor)
        ↓
Safe Robot Trajectory         (moveit_msgs/RobotTrajectory)
        ↓
Robot Controller              (Unity trajectory executor, with pause/abort gate)
        ↓
Unity Robotic Arm
```

### 15.3 Interaction with tomato detection

```
Camera
├── Tomato detection ──→ Tomato localization ──→ /tomato_position ─────────┐
│                                                                          │
└── Obstacle detection ──→ Obstacle localization ──→ /obstacles            │
                                   │                                       │
                                   ↓                                       ↓
                          scene_manager ──→ MoveIt planning scene ──→ harvest_fsm
                                                                          │
                                                       Path planning (goal = tomato,
                                                        constraints = obstacles)
                                                                          ↓
                                                                   Robot motion
                                                                          ↓
                                                                     Harvesting
```

The planner receives **both** the target (tomato pose) and the constraints (obstacles). One special case needs care: the tomato you want to grasp must **not** block its own approach. See §22.4.

### 15.4 Nodes and responsibilities

| Node | Runs in | Job |
|---|---|---|
| `ObstacleReporter` (C#) | Unity | Publishes ground-truth obstacle shape/pose (Option A, §18) |
| `MovingObstacle` (C#) | Unity | Simulated dynamic obstacle (person/box) |
| `ExecutionGate` + executor patch (C#) | Unity | Pause / resume / abort trajectory playback |
| `ClearanceProbe` (C#) | Unity | Collision count and min clearance for metrics |
| `obstacle_detection.py` | ROS | Stamps, filters, estimates velocity, expires stale obstacles |
| `scene_manager.py` | ROS | Mirrors `/obstacles` into MoveIt's planning scene, with safety inflation |
| `safety_monitor.py` | ROS | Reactive distance check + remaining-path validity check; commands PAUSE |
| `harvest_fsm.py` | ROS | State machine: plan, execute, stop, replan, harvest |
| `metrics_logger.py` | ROS | Records events and metrics to CSV |

---

## 16. Static Obstacle Avoidance

### 16.1 Choosing a representation (ROS 1)

| Approach | What it is | Fits static? | Fits ROS 1 + MoveIt 1? | Verdict |
|---|---|---|---|---|
| **Unity colliders** | Physics colliders in the scene | Yes, but only for Unity physics | MoveIt cannot see them | Needed for realism and collision *counting*; **not sufficient for planning** |
| **URDF collision geometry** | `<collision>` elements in the robot/workcell description | Yes, permanent items | Yes; requires editing URDF and regenerating/updating the MoveIt config | **Use for the permanent workcell**: table, walls, fixed rig, robot's own links |
| **Planning scene / MoveIt CollisionObjects** | `moveit_msgs/CollisionObject` primitives added at runtime | Yes | Yes; native, runtime add/move/remove | **Primary approach** for plant stems, leaves, test obstacles, basket |
| **Octomap** | Voxel map from `PointCloud2` / depth image | Yes for unknown clutter | Yes; needs a 3D sensor, tuning, and is updated slowly | Optional stretch (Option C/D in §18) |
| **Mesh collision objects** | Triangle meshes as CollisionObjects | Yes | Yes, but slower collision checks | Avoid; use primitives or convex approximations |
| **2D occupancy grid** (`nav_msgs/OccupancyGrid`) | Planar map | No | Designed for mobile-robot navigation | **Not appropriate** for a 6-DOF arm |

**Recommendation.** Static obstacles come from two sources:
1. **URDF** for things that never move (table, walls, robot frame).
2. **CollisionObject primitives** (box, cylinder, sphere) for everything else, generated from Unity ground truth.

Approximate leaves as boxes or spheres, stems as cylinders, the person model as a vertical cylinder or capsule (cylinder plus two spheres). Slight over-approximation is intentional; it is a safety margin.

### 16.2 Unity ↔ MoveIt geometry consistency

| Problem | Symptom | Fix |
|---|---|---|
| MoveIt shape smaller than the Unity collider | Planner says "free", Unity registers a collision | MoveIt geometry must be **at least as large** as the Unity collider |
| Unity collider is a mesh, MoveIt is a primitive | Small mismatches | Use the primitive's bounding shape; keep Unity colliders as simple primitives too |
| Unit or axis mix-ups | Obstacle appears rotated or mirrored in RViz | Use `ROSGeometry` (`To<FLU>()`); verify in RViz first (see checklist) |
| Frame mismatch | Everything offset | Publish obstacle poses **relative to the robot base** and set `frame_id` to the same frame MoveIt plans in |

### 16.3 Adding static collision objects to the planning scene

`scene_manager.py` (below) does this from `/obstacles`. For a hand-made test, use the verified `PlanningSceneInterface`:

```python
#!/usr/bin/env python3
import sys, rospy, moveit_commander
from geometry_msgs.msg import PoseStamped

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("add_static_box")
scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(1.0)                       # let publishers connect

p = PoseStamped()
p.header.frame_id = "base_link"        # placeholder: your planning frame
p.pose.position.x, p.pose.position.y, p.pose.position.z = 0.35, 0.0, 0.30
p.pose.orientation.w = 1.0
scene.add_box("test_box", p, size=(0.10, 0.30, 0.30))

# wait until move_group has actually received it
for _ in range(50):
    if "test_box" in scene.get_known_object_names():
        break
    rospy.sleep(0.1)
```

Look at RViz's MotionPlanning display (Scene Objects / planning scene display) to confirm it appears where Unity shows it.

### 16.4 Worked example: straight path blocked → detour

Setup: robot at home, tomato pre-grasp pose at about 0.6 m in front of the base, box placed roughly midway.

```python
#!/usr/bin/env python3
# demo_static_detour.py  — unexecuted reference code
import sys, copy, rospy, moveit_commander
from geometry_msgs.msg import PoseStamped

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("demo_static_detour")
group = moveit_commander.MoveGroupCommander("arm")           # placeholder group
scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(1.0)

target = group.get_current_pose().pose
target.position.x += 0.35                                    # placeholder: use your real pre-grasp pose

# 1. straight-line (Cartesian) attempt, collision-aware by default
group.set_start_state_to_current_state()
plan_line, fraction = group.compute_cartesian_path([copy.deepcopy(target)], 0.01, 0.0)
rospy.loginfo("Straight-line achievable fraction: %.2f", fraction)
# With the box in the way, fraction should be < 1.0  → straight path is NOT allowed.

# 2. joint-space sampling planner (OMPL) finds a detour
group.set_pose_target(target)
res = group.plan()
# Noetic: (success, trajectory, planning_time, error_code). VERIFY on your install;
# older releases return only the trajectory.
ok = res[0] if isinstance(res, tuple) else bool(res.joint_trajectory.points)
rospy.loginfo("OMPL plan success: %s", ok)
```

Expected observations: `fraction < 1.0` (the straight Cartesian line is rejected) and `ok == True` (OMPL routes around the box). In RViz, the displayed path bends around the obstacle. That bend **is** the "safe waypoint": OMPL produces intermediate joint configurations that keep every link clear.

To show an explicit `Robot → Safe waypoint → Tomato` version for the report, plan to a hand-chosen via-pose above the box first, then plan from there to the tomato:

```python
via = copy.deepcopy(target)
via.position.x = 0.30; via.position.z += 0.25   # above the box (placeholder numbers)
for goal in (via, target):
    group.set_start_state_to_current_state()
    group.set_pose_target(goal)
    # plan, publish to Unity, wait for completion, then continue
```

### 16.5 How collision checking works (what to say in your report)

1. **Geometry.** Robot links (from URDF collision elements) and world objects (CollisionObjects) are each a set of primitive shapes with poses.
2. **State validity.** For a candidate joint vector, forward kinematics places every link; MoveIt's collision checker (FCL by default in MoveIt 1 — `VERIFY` in your config) tests link-vs-world and link-vs-link pairs. The state is valid if there are no contacts.
3. **Allowed collision matrix (ACM).** Pairs listed in the SRDF/ACM are skipped. Adjacent links are excluded by default. This is why self-collision does not fire on neighbouring links, and why the gripper can be allowed to touch the tomato.
4. **Path validity.** A sampling planner such as OMPL validates *motions* by checking states at a resolution along each edge, so a path can be "valid at samples but clipping between them" if the resolution is too coarse. Keep obstacles inflated (§22) to absorb this.
5. **Result.** The planner returns a path of valid states; MoveIt time-parameterizes it into a `RobotTrajectory`.

### 16.6 Static-obstacle scene loader (part of `scene_manager.py`, §19.4)

Static obstacles are added once, and re-added only if they move more than about 1 cm. Dynamic obstacles are updated at a limited rate.

---

## 17. Dynamic Obstacle Avoidance

### 17.1 Why dynamic obstacles are harder

| Static | Dynamic |
|---|---|
| Position fixed; plan once | Position changes; a path that was safe a moment ago may be unsafe now |
| Obstacle map built once | Obstacle map must be **refreshed continuously** |
| One plan suffices | May need to **stop, wait, and replan** |
| Planning time is not critical | Detection + planning + stopping latency must fit inside the time before contact |

MoveIt 1 plans a whole trajectory and executes it; it is **not** a continuous reactive controller. So dynamic avoidance here means **monitor → stop/slow → replan → resume**, not smooth real-time evasion. Say this explicitly in your report.

### 17.2 Simulated dynamic obstacle in Unity

```csharp
// ObstacleTag.cs — attach to every obstacle GameObject
using UnityEngine;

public enum ObstacleShape : byte { Box = 1, Sphere = 2, Cylinder = 3 }

public class ObstacleTag : MonoBehaviour
{
    public string id = "obstacle_1";
    public ObstacleShape shape = ObstacleShape.Box;
    public bool isDynamic = false;
}
```

```csharp
// MovingObstacle.cs — ping-pongs between two points; start/stop for repeatable tests
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

[RequireComponent(typeof(Rigidbody))]
public class MovingObstacle : MonoBehaviour
{
    public Transform pointA, pointB;
    public float speed = 0.25f;               // m/s; tune per test
    public bool running = false;
    public string commandTopic = "/unity/scenario_cmd";   // "dyn_start" | "dyn_stop" | "dyn_reset"

    Rigidbody rb; Vector3 target; Vector3 startPos;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.isKinematic = true;                 // moved by script, still generates collisions
        startPos = rb.position; target = pointB.position;
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>(commandTopic, OnCmd);
    }

    void OnCmd(StringMsg m)
    {
        if (m.data == "dyn_start") running = true;
        else if (m.data == "dyn_stop") running = false;
        else if (m.data == "dyn_reset") { running = false; rb.position = startPos; target = pointB.position; }
    }

    void Update() { if (Input.GetKeyDown(KeyCode.M)) running = !running; }   // manual trigger

    void FixedUpdate()
    {
        if (!running) return;
        Vector3 next = Vector3.MoveTowards(rb.position, target, speed * Time.fixedDeltaTime);
        rb.MovePosition(next);
        if ((next - target).sqrMagnitude < 1e-4f)
            target = (target == pointB.position) ? pointA.position : pointB.position;
    }
}
```

Use a capsule/cylinder or a simple humanoid "person model" as the visual; tag it `ObstacleTag { id = "person_1", shape = Cylinder, isDynamic = true }`.

### 17.3 The seven required behaviours

| # | Requirement | Where implemented |
|---|---|---|
| 1 | Detect the moving obstacle | `ObstacleReporter` (ground truth) → `obstacle_detection` |
| 2 | Estimate current position | `ObstacleArray.pose` |
| 3 | Optionally estimate velocity | `obstacle_detection.py` (low-passed finite difference) |
| 4 | Publish/update position in ROS | `/obstacles`, and the planning scene via `scene_manager` |
| 5 | Check whether the current trajectory is still safe | `safety_monitor.py` (reactive distance + remaining-path validity) |
| 6 | Stop, slow down, or replan | `/harvest/exec_cmd` PAUSE, then `harvest_fsm` REPLAN |
| 7 | Resume harvesting when safe | FSM re-plans once the path validates, or waits and retries |

### 17.4 Simulation shortcut vs a real perception system

| | This project (first implementation) | A real system |
|---|---|---|
| Obstacle position | Read directly from Unity transforms (**ground truth**, exact, no noise) | Estimated from sensors, noisy, delayed, sometimes missing |
| Obstacle shape | Known exactly | Must be inferred (clustering, segmentation, tracking) |
| Velocity | Exact or finite-difference of exact positions | Filtered estimate (e.g. Kalman) with uncertainty |
| Failure modes | None from perception | Occlusion, false positives, calibration errors |

Write this in your report as a stated limitation. Your results demonstrate the **planning and safety logic**, not the perception.

---

## 18. Obstacle Sensors

### 18.1 Options compared

| | **A. Unity ground truth** | **B. RGB camera** | **C. Depth camera** | **D. LiDAR** |
|---|---|---|---|---|
| Realism | Low | Medium | Medium–high | Medium–high |
| Implementation effort | **Very low** | High (segmentation/detection needed; no distance) | Medium–high | Medium (raycast scan) |
| Gives 3D position | Yes (exact) | Not from one image | Yes | Yes |
| ROS message | Custom `ObstacleArray` | `sensor_msgs/Image` | `sensor_msgs/Image` (depth) or `PointCloud2` | `sensor_msgs/LaserScan` (2D) or `PointCloud2` (3D) |
| Works with MoveIt octomap | No (not needed) | No | **Yes** (Depth/PointCloud updater) | **Yes** (PointCloud2) |
| Hardware needed | None | None (simulated) | None (simulated) | None (simulated) |
| Main risk | "Not real perception" | Too much CV work for a project focused on manipulation | Unity depth publishing is custom work; octomap updates slowly | Same custom work; sparse for thin stems |

### 18.2 Recommendation for the university project

1. **Baseline (required, Stages 1–5): Option A.** It keeps the work on planning, safety and state machine, which is the actual assignment.
2. **Optional realism upgrade (only after Stage 5 works): a simulated "virtual depth/LiDAR" from Unity raycasts**, published as `sensor_msgs/PointCloud2`, feeding a small clustering node into the same `/obstacles` topic. Because `obstacle_detection` is the only node that changes, the rest of the system is unaffected.

I did **not** find a turn-key Unity depth-camera-to-`PointCloud2` publisher in the Hub materials I checked, so plan on writing it yourself (`Physics.Raycast` grid → point array → `PointCloud2`). No physical hardware is required for any option.

If you later want MoveIt to build an octomap itself, configure a `sensors` YAML with `occupancy_map_monitor/PointCloudOctomapUpdater` (fields: `point_cloud_topic`, `max_range`, `padding_offset`, `max_update_rate`, …) per the MoveIt perception tutorial. Note `max_update_rate` in the documented example is 1 Hz, which is why I recommend **explicit CollisionObjects for anything that moves quickly**.

---

## 19. Obstacle Representation in ROS

### 19.1 Which message types are appropriate

| Message | Appropriate here? | Reason |
|---|---|---|
| `geometry_msgs/PoseStamped` | **Yes**, for `/tomato_position` | One target pose in a frame |
| `geometry_msgs/Pose` | **Yes**, as a field inside our obstacle message | Pose alone lacks shape and velocity |
| `visualization_msgs/Marker(Array)` | **Yes, for RViz only** (`/obstacles/viz`) | Designed for display, not as a planning data interface |
| `sensor_msgs/PointCloud2` | Only if you adopt Option C/D | 3D sensor data; consumed by MoveIt octomap or a clustering node |
| `sensor_msgs/LaserScan` | Only for a 2D scan variant | Planar; poorly suited to obstacles in a 3D arm workspace |
| `moveit_msgs/CollisionObject` | **Yes**, as the *output* into MoveIt | This is what the planning scene consumes |
| `tf` / `tf2` | **Yes** | Frames: base ↔ camera ↔ links; needed to express detections in the planning frame |
| **Custom `harvest_msgs/Obstacle`** | **Yes** | No standard ROS 1 message carries *shape + pose + size + velocity + dynamic flag* together |

### 19.2 Custom message (`harvest_msgs` package)

```
# harvest_msgs/msg/Obstacle.msg
std_msgs/Header header
string id
uint8 shape                       # 1=BOX 2=SPHERE 3=CYLINDER (same numbers as SolidPrimitive/Marker convention)
bool is_dynamic
geometry_msgs/Pose pose           # in header.frame_id
geometry_msgs/Vector3 size        # BOX: x,y,z extents | SPHERE: x = diameter | CYLINDER: x = diameter, z = height (axis = z)
geometry_msgs/Vector3 velocity    # m/s; zero if static or unknown
float32 confidence                # 1.0 for ground truth
```

```
# harvest_msgs/msg/ObstacleArray.msg
std_msgs/Header header
harvest_msgs/Obstacle[] obstacles   # FULL snapshot of currently visible obstacles
```

`package.xml`/`CMakeLists.txt`: standard `message_generation` with dependencies `std_msgs geometry_msgs`. Build in the catkin workspace, `source devel/setup.bash`.

**Unity side:** use `Robotics → Generate ROS Messages…` and browse to the workspace `src` folder containing `harvest_msgs` (same flow the Hub tutorial uses for its custom `MoverService`). The `_msgs` suffix is typically dropped in the generated C# namespace, so expect `using RosMessageTypes.Harvest;` and classes `ObstacleMsg`, `ObstacleArrayMsg`. `VERIFY` the actual namespace in the generated folder. ROS-TCP-Endpoint must be able to import `harvest_msgs`, so the workspace must be built and sourced before launching it.

### 19.3 Topic map

```
/camera/image_raw ─────────────→ [tomato_detection] ──→ /tomato_position  (PoseStamped)

/unity/obstacles_gt (ObstacleArray, Option A ground truth)
        ↓
[obstacle_detection]  stamps, filters, velocity, expiry
        ↓
/obstacles (ObstacleArray)  ─┬─→ [scene_manager] ─→ /collision_object → move_group planning scene
                             ├─→ [safety_monitor]
                             └─→ /obstacles/viz (MarkerArray) → RViz
```

Full table of every topic in the system is in §27.2.

### 19.4 Unity → ROS: `ObstacleReporter.cs`

```csharp
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Harvest;       // VERIFY namespace after message generation
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class ObstacleReporter : MonoBehaviour
{
    public Transform robotBase;                     // GameObject that corresponds to ROS base_link
    public string frameId = "base_link";
    public string topic = "/unity/obstacles_gt";
    public float publishHz = 20f;

    ROSConnection ros;
    readonly List<ObstacleTag> tags = new List<ObstacleTag>();
    float nextTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ObstacleArrayMsg>(topic);
    }

    void Update()
    {
        if (Time.time < nextTime) return;
        nextTime = Time.time + 1f / publishHz;

        tags.Clear();
        tags.AddRange(FindObjectsOfType<ObstacleTag>());   // or FindObjectsByType on newer Unity; only ACTIVE objects are reported,
                                                           // so SetActive(false) = "obstacle disappeared"
        var arr = new ObstacleArrayMsg();
        arr.header = new HeaderMsg { frame_id = frameId }; // stamp is added on the ROS side
        arr.obstacles = new ObstacleMsg[tags.Count];
        for (int i = 0; i < tags.Count; i++) arr.obstacles[i] = Build(tags[i]);
        ros.Publish(topic, arr);
    }

    ObstacleMsg Build(ObstacleTag o)
    {
        Transform t = o.transform;
        Vector3 p = robotBase.InverseTransformPoint(t.position);                 // relative to robot base
        Quaternion q = Quaternion.Inverse(robotBase.rotation) * t.rotation;
        Vector3 s = t.lossyScale;

        // Unity → ROS(FLU) axis permutation: ROS x = Unity z, ROS y = Unity x, ROS z = Unity y.
        // Unity primitives: Cube 1x1x1, Sphere diameter 1, Cylinder diameter 1 and HEIGHT 2 (along local Y).
        Vector3Msg size;
        switch (o.shape)
        {
            case ObstacleShape.Box:      size = new Vector3Msg(s.z, s.x, s.y); break;
            case ObstacleShape.Sphere:   size = new Vector3Msg(s.x, 0, 0); break;
            default /*Cylinder*/:        size = new Vector3Msg(s.x, 0, 2f * s.y); break;
        }

        return new ObstacleMsg
        {
            header = new HeaderMsg { frame_id = frameId },
            id = o.id,
            shape = (byte)o.shape,
            is_dynamic = o.isDynamic,
            pose = new PoseMsg { position = p.To<FLU>(), orientation = q.To<FLU>() },  // VERIFY implicit conversions in your ROSGeometry version
            size = size,
            velocity = new Vector3Msg(0, 0, 0),
            confidence = 1f
        };
    }
}
```

The obstacle GameObjects must be Unity primitives (or have known scale) for the size logic to be correct. If you use custom meshes, add explicit `size` fields to `ObstacleTag` instead of reading `lossyScale`.

### 19.5 ROS: `obstacle_detection.py`

```python
#!/usr/bin/env python3
# Ground-truth passthrough + velocity estimation + expiry.
import rospy
from harvest_msgs.msg import ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray

ALPHA = 0.5          # velocity low-pass factor (0..1); higher = more responsive, noisier

class Track:
    def __init__(self): self.p = None; self.t = None; self.v = [0.0, 0.0, 0.0]

tracks = {}

def cb(msg):
    now = rospy.Time.now()                        # ROS-side stamp: also the latency reference clock
    out = ObstacleArray(); out.header.stamp = now; out.header.frame_id = msg.header.frame_id
    viz = MarkerArray()
    for o in msg.obstacles:
        tr = tracks.setdefault(o.id, Track())
        p = o.pose.position
        if o.is_dynamic and tr.t is not None:
            dt = (now - tr.t).to_sec()
            if dt > 1e-3:
                raw = [(p.x - tr.p[0]) / dt, (p.y - tr.p[1]) / dt, (p.z - tr.p[2]) / dt]
                tr.v = [ALPHA * r + (1 - ALPHA) * v for r, v in zip(raw, tr.v)]
        tr.p = [p.x, p.y, p.z]; tr.t = now
        o.header.stamp = now
        o.velocity.x, o.velocity.y, o.velocity.z = tr.v if o.is_dynamic else (0.0, 0.0, 0.0)
        out.obstacles.append(o)
        viz.markers.append(to_marker(o))
    # forget tracks of obstacles that vanished
    for k in [k for k in tracks if k not in {o.id for o in msg.obstacles}]:
        del tracks[k]
    pub.publish(out); pub_viz.publish(viz)

def to_marker(o):
    m = Marker(); m.header = o.header; m.ns = "dynamic" if o.is_dynamic else "static"
    m.id = abs(hash(o.id)) % 100000; m.type = o.shape; m.action = Marker.ADD
    m.pose = o.pose
    m.scale.x = o.size.x; m.scale.y = o.size.y or o.size.x; m.scale.z = o.size.z or o.size.x
    m.color.r, m.color.g, m.color.b, m.color.a = (1, .3, .1, .6) if o.is_dynamic else (.2, .5, 1, .6)
    m.lifetime = rospy.Duration(0.5)
    return m

if __name__ == "__main__":
    rospy.init_node("obstacle_detection")
    pub = rospy.Publisher("/obstacles", ObstacleArray, queue_size=1)
    pub_viz = rospy.Publisher("/obstacles/viz", MarkerArray, queue_size=1)
    rospy.Subscriber("/unity/obstacles_gt", ObstacleArray, cb, queue_size=1)
    rospy.spin()
```

Velocity here is computed on ROS receive times, so TCP jitter adds noise; the low-pass filter absorbs most of it. If you need cleaner velocity, stamp messages in Unity with simulation time.

### 19.6 ROS: `scene_manager.py` (planning-scene synchronization)

```python
#!/usr/bin/env python3
import sys, math, rospy, moveit_commander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float32
from harvest_msgs.msg import ObstacleArray

class SceneManager:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("scene_manager")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.safety = rospy.get_param("~safety_distance", 0.10)      # m — SIMULATION PARAMETER, tune it (§22)
        self.horizon = rospy.get_param("~prediction_horizon", 1.0)   # s, for dynamic inflation
        self.dyn_max_hz = rospy.get_param("~dynamic_update_hz", 5.0)
        self.move_eps = 0.01                                         # m, static re-add threshold
        self.exclude = set()                                         # ids not to insert (e.g. current target tomato)
        self.known = {}                                              # id -> (last_pose_xyz, last_time)
        rospy.Subscriber("/harvest/safety_distance", Float32, lambda m: self.set_margin(m.data))
        rospy.Subscriber("/harvest/exclude_obstacle", __import__("std_msgs.msg").msg.String,
                         lambda m: self.exclude.add(m.data))
        rospy.Subscriber("/obstacles", ObstacleArray, self.cb, queue_size=1)
        self.last = None

    def set_margin(self, v):
        self.safety = v
        self.known.clear()                        # force re-inflation of everything
        if self.last: self.cb(self.last)

    def to_co(self, o):
        speed = math.sqrt(o.velocity.x**2 + o.velocity.y**2 + o.velocity.z**2)
        m = self.safety + (speed * self.horizon if o.is_dynamic else 0.0)   # constant-velocity inflation
        co = CollisionObject()
        co.id = o.id; co.header.frame_id = o.header.frame_id
        co.operation = CollisionObject.ADD        # ADD on an existing id replaces it (per moveit_msgs docs; VERIFY)
        s = SolidPrimitive()
        if o.shape == 1:
            s.type = SolidPrimitive.BOX
            s.dimensions = [o.size.x + 2*m, o.size.y + 2*m, o.size.z + 2*m]
        elif o.shape == 2:
            s.type = SolidPrimitive.SPHERE
            s.dimensions = [o.size.x / 2.0 + m]
        else:
            s.type = SolidPrimitive.CYLINDER
            s.dimensions = [o.size.z + 2*m, o.size.x / 2.0 + m]     # [height, radius]
        co.primitives = [s]; co.primitive_poses = [o.pose]
        return co

    def cb(self, msg):
        self.last = msg
        now = rospy.Time.now().to_sec()
        seen = set()
        for o in msg.obstacles:
            if o.id in self.exclude: continue
            seen.add(o.id)
            p = (o.pose.position.x, o.pose.position.y, o.pose.position.z)
            prev = self.known.get(o.id)
            if prev is not None:
                moved = math.dist(p, prev[0]) > self.move_eps
                if not o.is_dynamic and not moved: continue
                if o.is_dynamic and (now - prev[1]) < 1.0 / self.dyn_max_hz: continue
            self.scene.add_object(self.to_co(o))
            self.known[o.id] = (p, now)
        for oid in [k for k in self.known if k not in seen]:      # obstacle disappeared
            self.scene.remove_world_object(oid)
            del self.known[oid]

if __name__ == "__main__":
    SceneManager(); rospy.spin()
```

`math.dist` needs Python 3.8+ (fine on Noetic). Note: a **dynamic obstacle's swept prediction is conservative but simple**; the inflation by `speed × horizon` makes it "fat" in all directions.

---

## 20. Path Planning

### 20.1 Motion control vs path planning

| | Motion control | Path planning |
|---|---|---|
| Question | "Move to this pose / follow this trajectory" | "Find a safe route to this pose without hitting anything" |
| Needs a world model? | No | **Yes** (planning scene) |
| Output | Actuator commands | A collision-free trajectory |
| In this project | Unity executor moves articulation drives along the trajectory | MoveIt / OMPL |

The project requires **both**. Unity's executor alone (a straight-line interpolation) would drive through obstacles.

### 20.2 Is MoveIt appropriate?

**Yes, if** your arm has a URDF and a MoveIt configuration (Setup Assistant) with a working planning group. The Unity Robotics Hub pick-and-place tutorial demonstrates precisely this combination (URDF import, MoveIt planning service, Unity executes the returned trajectory) on ROS Noetic, so you are following a supported path. Caveats:

- MoveIt 1 is a **planner**, not a real-time reactive controller (see §17.1).
- Arms with fewer than 6 DOF cannot reach arbitrary 6D poses; use position-only goals or joint goals for those.
- Planning success depends on a reachable, collision-free goal: the goal pose must itself be outside all inflated obstacles.

### 20.3 MoveIt concepts, mapped to this project

| Concept | Meaning | Where you touch it |
|---|---|---|
| **Planning group** | The set of joints being planned (`arm`) | `MoveGroupCommander("arm")` |
| **Planning scene** | Robot + world + attached objects | `scene_manager`, `PlanningSceneInterface` |
| **Collision objects** | Obstacles as shapes | `CollisionObject` messages |
| **Robot state** | Joint positions at the start of a plan | `set_start_state_to_current_state()` fed by `/joint_states` from Unity |
| **Joint limits** | Position/velocity limits from URDF | Respected automatically |
| **Self-collision** | Link-vs-link checking | SRDF collision matrix |
| **End-effector target** | Goal pose | `set_pose_target(pose)` |
| **Collision checking** | Validity of states/edges | §16.5 |
| **Trajectory generation** | Sampling planner path → time-parameterized trajectory | `plan()` |
| **Replanning** | New plan from the *current* state after the world changed | FSM `REPLAN` state (§21) |

Planner settings to start with (all standard `MoveGroupCommander` calls): `set_planning_time(3–5)`, `set_num_planning_attempts(5–10)`, `set_max_velocity_scaling_factor(0.2–0.4)`. The default MoveIt-config planner (typically `RRTConnectkConfigDefault` in `ompl_planning.yaml`; `VERIFY`) is a good first choice. Slower speeds also shorten stopping distance (§22).

### 20.4 Data flow with Unity

```
harvest_fsm ──plan()──→ RobotTrajectory ──/harvest/trajectory──→ Unity executor
                                                                     │ plays joint targets on ArticulationBody drives
Unity ──/joint_states (30–50 Hz)──→ robot_state_publisher → /tf
Unity ──/harvest/exec_status ("DONE"/"PAUSED"/"ABORTED")──→ harvest_fsm
harvest_fsm / safety_monitor ──/harvest/exec_cmd ("PAUSE"/"RESUME"/"ABORT")──→ Unity
```

Unity needs the C# class for `moveit_msgs/RobotTrajectory` (the Hub pick-and-place tutorial generates it as the first message). `/joint_states` must be published continuously so `move_group` and `tf` know the real current state; replanning starts from that state.

### 20.5 Simpler alternative if MoveIt turns out to be impractical

Keep the same architecture (`obstacle map → planner → trajectory → executor`), but replace MoveIt with a small **joint-space planner** you control:

1. Write your own collision check: approximate each link as 2–3 spheres placed via forward kinematics; obstacles as inflated primitives (the same distance functions used in `safety_monitor.py`).
2. Plan with **RRT / RRT-Connect in joint space** (a few hundred lines) or a **via-point search** (try a small set of candidate via-poses above/left/right of the blocking obstacle, keep the first whose two segments are collision-free).
3. Output a `trajectory_msgs/JointTrajectory` to the same Unity executor.

This is less capable than OMPL and needs your own forward kinematics (from the URDF via KDL/your own code), but it is fully under your control and easy to explain. Treat it as a fallback, not the plan.

---

## 21. Dynamic Replanning

### 21.1 The loop (10 required steps)

```
 1. Detect tomato                       (tomato_detection)
 2. Localize tomato                     (/tomato_position)
 3. Detect obstacles                    (/obstacles)
 4. Plan collision-free path            (MoveIt, scene with inflated obstacles)
 5. Start robot motion                  (publish /harvest/trajectory)
 6. Continuously monitor obstacles      (safety_monitor, ≥ 20 Hz)
 7. If an obstacle enters the safety zone:
        – PAUSE the robot               (safety_monitor → /harvest/exec_cmd, direct, low latency)
        – update obstacle information   (short settle time)
        – replan from current state
        – continue only if the new path validates
 8. Reach tomato
 9. Harvest
10. Return safely
```

### 21.2 Two-layer safety (why both)

| Layer | Question | Method | Speed |
|---|---|---|---|
| **Reactive** | "Is anything dangerously close *right now*?" | Distance from robot link spheres (from `tf`) to obstacle shapes (+ predicted position) | Fast (tens of Hz) |
| **Path validity** | "Will the *rest of my planned path* hit the updated scene?" | Sample remaining waypoints, call MoveIt's `/check_state_validity` (`moveit_msgs/GetStateValidity`; `VERIFY` service name via `rosservice list`) | Slower (a few Hz) |

The reactive layer protects the arm immediately; the path-validity layer decides whether the current plan is still usable and triggers a replan *before* the arm gets close.

### 21.3 `safety_monitor.py`

```python
#!/usr/bin/env python3
# Unexecuted reference code.
import json, math, rospy, tf2_ros, numpy as np
from tf.transformations import quaternion_matrix
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory, RobotState
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from harvest_msgs.msg import ObstacleArray

# ---- configuration (rosparam in real use) ----
STOP_DIST  = 0.10   # m clearance (surface-to-surface) that triggers PAUSE  — tune (§22)
SLOW_DIST  = 0.25   # m clearance that would trigger slow-down
HORIZON    = 0.5    # s look-ahead for dynamic obstacles
LINKS = [("forearm_link", 0.06), ("wrist_link", 0.05), ("tool_link", 0.08)]   # (tf frame, sphere radius) PLACEHOLDERS
BASE = "base_link"
GROUP = "arm"

def dist_to_obstacle(pt, o, shift=(0, 0, 0)):
    """Signed-ish distance from point to obstacle surface (>=0 outside)."""
    pos = np.array([o.pose.position.x, o.pose.position.y, o.pose.position.z]) + np.array(shift)
    q = o.pose.orientation
    R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    l = R.T.dot(np.asarray(pt) - pos)                        # point in obstacle frame
    if o.shape == 1:                                         # box
        half = np.array([o.size.x, o.size.y, o.size.z]) / 2
        return float(np.linalg.norm(np.maximum(np.abs(l) - half, 0)))
    if o.shape == 2:                                         # sphere
        return float(max(np.linalg.norm(l) - o.size.x / 2, 0))
    dxy = math.hypot(l[0], l[1]) - o.size.x / 2              # cylinder, axis = z
    dz = abs(l[2]) - o.size.z / 2
    return float(math.hypot(max(dxy, 0), max(dz, 0)))

class Monitor:
    def __init__(self):
        rospy.init_node("safety_monitor")
        self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf)
        self.obs = []; self.traj = None; self.joints = None
        self.state = "SAFE"
        self.cmd = rospy.Publisher("/harvest/exec_cmd", String, queue_size=5)
        self.status = rospy.Publisher("/harvest/safety", String, queue_size=5)
        self.events = rospy.Publisher("/harvest/events", String, queue_size=50)
        rospy.Subscriber("/obstacles", ObstacleArray, lambda m: setattr(self, "obs", m.obstacles))
        rospy.Subscriber("/harvest/trajectory", RobotTrajectory, lambda m: setattr(self, "traj", m))
        rospy.Subscriber("/joint_states", JointState, lambda m: setattr(self, "joints", m))
        rospy.wait_for_service("/check_state_validity")
        self.valid = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
        rospy.Timer(rospy.Duration(0.05), self.reactive)     # 20 Hz
        rospy.Timer(rospy.Duration(0.30), self.path_check)   # ~3 Hz

    def event(self, name, **kw):
        kw.update(event=name, t=rospy.Time.now().to_sec()); self.events.publish(json.dumps(kw))

    def reactive(self, _):
        worst = 1e9; who = None
        for frame, r in LINKS:
            try:
                tr = self.buf.lookup_transform(BASE, frame, rospy.Time(0), rospy.Duration(0.02)).transform.translation
            except Exception:
                continue
            pt = (tr.x, tr.y, tr.z)
            for o in self.obs:
                pred = (o.velocity.x * HORIZON, o.velocity.y * HORIZON, o.velocity.z * HORIZON) if o.is_dynamic else (0, 0, 0)
                d = min(dist_to_obstacle(pt, o), dist_to_obstacle(pt, o, pred)) - r
                if d < worst: worst, who = d, o.id
        new = "STOP" if worst < STOP_DIST else ("SLOW" if worst < SLOW_DIST else "SAFE")
        if new != self.state:
            self.event("safety_" + new.lower(), obstacle=who, clearance=worst)
            if new == "STOP": self.cmd.publish("PAUSE")       # direct fast path; FSM is informed via /harvest/safety
            self.state = new
        self.status.publish(self.state)

    def path_check(self, _):
        if self.traj is None or self.joints is None or not self.traj.joint_trajectory.points: return
        jt = self.traj.joint_trajectory
        cur = dict(zip(self.joints.name, self.joints.position))
        try:
            now_vec = [cur[n] for n in jt.joint_names]
        except KeyError:
            return
        idx = min(range(len(jt.points)),
                  key=lambda i: sum((a - b) ** 2 for a, b in zip(jt.points[i].positions, now_vec)))
        remaining = jt.points[idx::max(1, (len(jt.points) - idx) // 15)]      # ≈15 samples
        for pt in remaining:
            req = GetStateValidityRequest(); req.group_name = GROUP
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = list(jt.joint_names)
            req.robot_state.joint_state.position = list(pt.positions)
            if not self.valid(req).valid:
                self.event("path_invalid"); self.status.publish("PATH_INVALID"); self.cmd.publish("PAUSE"); return

if __name__ == "__main__":
    Monitor(); rospy.spin()
```

Placeholders that **must** be changed: link frame names/radii (use your actual arm's frames and a radius that covers each link) and thresholds. `GetStateValidity` has no protection against trajectories with a different joint-name order than the model; that is handled above by using the trajectory's own names.

### 21.4 Unity: pause/resume/abort gate

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ExecutionGate : MonoBehaviour
{
    public static bool Paused, AbortRequested;
    public string cmdTopic = "/harvest/exec_cmd", statusTopic = "/harvest/exec_status";
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(cmdTopic, m =>
        {
            if (m.data == "PAUSE") { Paused = true; ros.Publish(statusTopic, new StringMsg("PAUSED")); }
            else if (m.data == "RESUME") Paused = false;
            else if (m.data == "ABORT") { AbortRequested = true; Paused = false; }
        });
        ros.RegisterPublisher<StringMsg>(statusTopic);
    }
    public static void ReportDone() =>
        ROSConnection.GetOrCreateInstance().Publish("/harvest/exec_status", new StringMsg("DONE"));
}
```

Patch your trajectory-playback coroutine (in the Hub pick-and-place tutorial this is the trajectory-execution loop that steps through the `RobotTrajectory` points; check your copy for the exact script name) as follows:

```csharp
foreach (var point in trajectory.points)
{
    while (ExecutionGate.Paused && !ExecutionGate.AbortRequested)
        yield return null;                                   // hold current drive targets
    if (ExecutionGate.AbortRequested) { ExecutionGate.AbortRequested = false; yield break; }
    // ... existing code: set ArticulationBody drive targets from `point`, wait for it ...
}
ExecutionGate.ReportDone();
```

Also subscribe to `/harvest/trajectory` and start a new playback (aborting any current one) whenever a trajectory arrives. Register the "PAUSED" status only once per event to avoid flooding.

Realism note: an `ArticulationBody` drive holding its last target stops **instantly**; a real arm has a braking distance. That is why the safety distance includes a velocity term (§22.2).

### 21.5 Not a safety-rated system

This is a simulation-grade monitor. It has no redundancy, no certified stopping performance, no verified sensor integrity, and it depends on network latency and a Python process. Industrial collaborative robot safety is governed by standards such as ISO 10218 and ISO/TS 15066, using safety-rated monitored stop, speed-and-separation monitoring, or power-and-force limiting with certified hardware. **Do not claim your system meets these**; describe it as a functional demonstration of the *logic*.

---

## 22. Safety Zones

### 22.1 Layers of margin

```
             ┌───────── dynamic prediction margin  (speed × horizon)
             │   ┌───── SAFETY_DISTANCE (configurable)
             │   │   ┌─ robot link radius (link sphere/capsule)
             ▼   ▼   ▼
obstacle surface ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ →  planned link centreline must stay outside all of this
```

| Element | Meaning | Where in this design |
|---|---|---|
| **Collision geometry** | The real shape MoveIt/Unity uses | URDF collision elements, obstacle primitives |
| **Robot link dimensions** | Links are volumes, not lines | `LINKS` radii in `safety_monitor`; URDF geometry in MoveIt |
| **Safety margin** | Extra clearance so paths never merely touch | Obstacle inflation in `scene_manager` (`safety_distance`) |
| **End-effector clearance** | Tool is the closest thing to the plant | Larger sphere radius for `tool_link`; separate approach margin (§22.4) |
| **Dynamic prediction** | Obstacle keeps moving during planning + execution | Inflation by `speed × prediction_horizon`, and predicted position in the reactive check |

### 22.2 Configurable parameter, not a universal constant

```yaml
# config/safety.yaml
safety_distance: 0.10          # m   SIMULATION PARAMETER — tune (start here, do not treat as correct)
approach_margin: 0.02          # m   reduced margin during final approach to the tomato
stop_distance: 0.10            # m   reactive PAUSE threshold (clearance)
slow_distance: 0.25            # m
prediction_horizon: 1.0        # s   scene inflation for dynamic obstacles
monitor_horizon: 0.5           # s   reactive look-ahead
max_ee_speed: 0.30             # m/s planned tool speed limit (via velocity scaling)
```

**How to tune it:** `0.10 m` is only a starting value. Choose it from measurements:

```
required_clearance ≥ tool/link radius error
                   + v_max × (detection latency + command latency + loop period)   ← stop distance
                   + planner discretization slack
```

Illustrative example (invented numbers, replace with your measured latencies): with `v_max = 0.3 m/s` and a measured total reaction time of `0.15 s`, the reaction distance is `0.045 m`, so `0.10 m` leaves about `0.055 m` for the other terms. If your measured latency is 0.5 s, the same margin is inadequate. Metrics in §26 give you the latency numbers to justify the final value.

Too small → near-collisions in Test 3/5. Too large → planner reports "no path" in the dense tomato plant (Test 7 becomes the common case). Report the trade-off with the measured curve.

### 22.3 Dynamic obstacle prediction

Constant-velocity model: `p(t + τ) = p + v·τ`. The scene inflation `speed × horizon` is a conservative, simple approximation. It fails for obstacles that change direction abruptly; the reactive layer (20 Hz, using current position) is the backstop. State this limitation.

### 22.4 The tomato and its immediate surroundings

The tomato you want to grasp, and the stem right next to it, will always be **inside** a 10 cm safety margin of the approach path. Handle it in three steps:

1. **Transit phase** (home → pre-grasp pose ~0.10 m from the tomato): full `safety_distance`, all obstacles including the target tomato inserted.
2. **Before the final approach:** send `/harvest/exclude_obstacle` with the target's id so `scene_manager` stops inserting it, call `scene.remove_world_object(target_id)`, and publish `/harvest/safety_distance = approach_margin` (e.g. 0.02 m).
3. **Final approach:** a short Cartesian segment (`compute_cartesian_path`, small `eef_step`). If `fraction < 0.95`, treat the approach as blocked (replan or safe failure).

After grasping, attach the tomato to the gripper (`PlanningSceneInterface.attach_object` / `attach_box` with `touch_links` set to the gripper links) so the planner accounts for its size during the retreat; detach at release.

---

## 23. Robot State Machine

### 23.1 Diagram

```
IDLE
 ↓ (start)
SEARCHING_FOR_TOMATO ←──────────────────────────────────────────────┐
 ↓ (tomato pose received, fresh)                                     │
TOMATO_DETECTED                                                      │
 ↓                                                                   │
LOCALIZING_TOMATO   (average N samples, reject if unstable)          │
 ↓                                                                   │
CHECKING_PATH       (log straight-line result; check obstacle map)   │
 ↓                                                                   │
PLANNING_PATH ── fail after retries ──→ FAILED_SAFE                  │
 ↓ (plan ok, published to Unity)                                     │
MOVING_TO_TOMATO ── arrived ──→ APPROACHING_TOMATO                   │
 │                                                                   │
 └─ OBSTACLE_DETECTED? (safety=STOP / PATH_INVALID / exec PAUSED)    │
       YES → STOP → UPDATE_OBSTACLE_MAP → REPLAN ─ plan ok ─→ MOVING_TO_TOMATO
                                              └─ no path, timeout ─→ FAILED_SAFE
       NO  → continue
APPROACHING_TOMATO ─ blocked ─→ REPLAN / FAILED_SAFE
 ↓
GRASPING
 ↓
HARVESTING
 ↓
PLACING_TOMATO
 ↓
RETURNING ────────────────────────────────────────────────────────────┘
```

`FAILED_SAFE` is an addition (required by Test 7): the arm holds still, reports the reason, and either waits for operator reset or returns to `SEARCHING_FOR_TOMATO` after a cool-down.

### 23.2 Every state

| State | What happens | Exit condition |
|---|---|---|
| `IDLE` | Everything initialized, robot at home, waiting for start | Start command → `SEARCHING_FOR_TOMATO` |
| `SEARCHING_FOR_TOMATO` | Wait for a fresh `/tomato_position` (age < 1 s) | Tomato seen → `TOMATO_DETECTED` |
| `TOMATO_DETECTED` | Latch the candidate tomato id/pose | Immediately → `LOCALIZING_TOMATO` |
| `LOCALIZING_TOMATO` | Average N pose samples; reject if spread > threshold (e.g. 1 cm) | Stable → `CHECKING_PATH`; unstable → back to `SEARCHING` |
| `CHECKING_PATH` | Ensure obstacle map is fresh; record if the straight line is blocked (`compute_cartesian_path` fraction) | → `PLANNING_PATH` |
| `PLANNING_PATH` | `plan()` to the pre-grasp pose in the current scene; publish trajectory to Unity | Success → `MOVING_TO_TOMATO`; retries exhausted → `FAILED_SAFE` |
| `MOVING_TO_TOMATO` | Wait for Unity `DONE`; watch `/harvest/safety` and `exec_status` | `DONE` → `APPROACHING_TOMATO`; `STOP`/`PATH_INVALID`/`PAUSED` → obstacle branch |
| `OBSTACLE_DETECTED?` | Decision point: safe → keep moving, unsafe → `STOP` | see left |
| `STOP` | Ensure `PAUSE` sent; robot holds position | → `UPDATE_OBSTACLE_MAP` |
| `UPDATE_OBSTACLE_MAP` | Wait a short settle time (e.g. 0.5 s) for fresh `/obstacles` and scene update | → `REPLAN` |
| `REPLAN` | Plan from the **current** joint state; validate the path | Success → send `ABORT` for old trajectory, publish new one → `MOVING_TO_TOMATO`; failure → wait/retry until timeout → `FAILED_SAFE` |
| `APPROACHING_TOMATO` | Exclude target from scene, reduce margin, short Cartesian approach | Reached → `GRASPING`; blocked → `REPLAN`/`FAILED_SAFE` |
| `GRASPING` | Close gripper, confirm contact/closure; attach tomato to planning scene | Confirmed → `HARVESTING`; failed → retry/`FAILED_SAFE` |
| `HARVESTING` | Detach motion (pull/twist/short retreat from your section 1–14 design) with attached-object collision checking | Detached → `PLACING_TOMATO` |
| `PLACING_TOMATO` | Plan to the basket pose (basket is a static obstacle; target is the point above it), open gripper, detach object | Released → `RETURNING` |
| `RETURNING` | Plan back to a home joint configuration | Home → `SEARCHING_FOR_TOMATO` |
| `FAILED_SAFE` | Robot stationary; reason logged; obstacle map still monitored | Operator reset or timeout → `SEARCHING_FOR_TOMATO` |

Manipulation states (`GRASPING`, `HARVESTING`, `PLACING_TOMATO`) depend on your gripper design from the earlier sections. Obstacle checking still applies: keep the monitor running and use the same `MOVING`/`STOP`/`REPLAN` loop for every planned motion.

### 23.3 Skeleton implementation

```python
#!/usr/bin/env python3
# harvest_fsm.py — unexecuted skeleton. Motion states share one generic MOVING loop.
import sys, enum, json, rospy, moveit_commander
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory

class S(enum.Enum):
    IDLE = 0; SEARCHING_FOR_TOMATO = 1; TOMATO_DETECTED = 2; LOCALIZING_TOMATO = 3
    CHECKING_PATH = 4; PLANNING_PATH = 5; MOVING_TO_TOMATO = 6; STOP = 7
    UPDATE_OBSTACLE_MAP = 8; REPLAN = 9; APPROACHING_TOMATO = 10; GRASPING = 11
    HARVESTING = 12; PLACING_TOMATO = 13; RETURNING = 14; FAILED_SAFE = 15

class Harvest:
    MAX_REPLAN_WAIT = 20.0          # s to wait for an obstacle to clear before FAILED_SAFE
    MAP_SETTLE = 0.5                # s

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("harvest_fsm")
        self.group = moveit_commander.MoveGroupCommander(rospy.get_param("~group", "arm"))
        self.group.set_planning_time(3.0); self.group.set_num_planning_attempts(5)
        self.group.set_max_velocity_scaling_factor(0.3)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.state = S.IDLE; self.tomato = None; self.exec_status = None; self.safety = "SAFE"
        self.goal = None; self.after_move = None
        self.traj_pub = rospy.Publisher("/harvest/trajectory", RobotTrajectory, queue_size=1)
        self.cmd_pub = rospy.Publisher("/harvest/exec_cmd", String, queue_size=5)
        self.events = rospy.Publisher("/harvest/events", String, queue_size=50)
        self.margin_pub = rospy.Publisher("/harvest/safety_distance", Float32, queue_size=1, latch=True)
        rospy.Subscriber("/tomato_position", PoseStamped, lambda m: setattr(self, "tomato", m))
        rospy.Subscriber("/harvest/exec_status", String, lambda m: setattr(self, "exec_status", m.data))
        rospy.Subscriber("/harvest/safety", String, lambda m: setattr(self, "safety", m.data))

    def ev(self, name, **kw):
        kw.update(event=name, state=self.state.name, t=rospy.Time.now().to_sec())
        self.events.publish(json.dumps(kw))

    def go(self, s): self.ev("state_change", to=s.name); self.state = s

    def plan_to(self, pose):
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose)
        res = self.group.plan()                               # VERIFY return type on your MoveIt version
        ok, plan = (res[0], res[1]) if isinstance(res, tuple) else (bool(res.joint_trajectory.points), res)
        return plan if ok and plan.joint_trajectory.points else None

    def send(self, plan):
        self.exec_status = None
        self.traj_pub.publish(plan)

    def run(self):
        rate = rospy.Rate(20)
        replan_deadline = None
        self.margin_pub.publish(rospy.get_param("~safety_distance", 0.10))
        while not rospy.is_shutdown():
            s = self.state
            if s == S.IDLE:
                self.go(S.SEARCHING_FOR_TOMATO)
            elif s == S.SEARCHING_FOR_TOMATO:
                if self.tomato and (rospy.Time.now() - self.tomato.header.stamp).to_sec() < 1.0:
                    self.go(S.TOMATO_DETECTED)
            elif s == S.TOMATO_DETECTED:
                self.go(S.LOCALIZING_TOMATO)
            elif s == S.LOCALIZING_TOMATO:
                # TODO: average N samples; reject if spread > 1 cm. Then:
                self.goal = self.pregrasp(self.tomato); self.go(S.CHECKING_PATH)
            elif s == S.CHECKING_PATH:
                # TODO: compute_cartesian_path fraction → self.ev("straight_line", fraction=...)
                self.go(S.PLANNING_PATH)
            elif s == S.PLANNING_PATH:
                plan = self.plan_to(self.goal)
                if plan is None:
                    self.ev("no_plan"); self.go(S.FAILED_SAFE)
                else:
                    self.send(plan); self.after_move = S.APPROACHING_TOMATO; self.go(S.MOVING_TO_TOMATO)
            elif s == S.MOVING_TO_TOMATO:
                if self.exec_status == "DONE":
                    self.go(self.after_move)
                elif self.safety in ("STOP", "PATH_INVALID") or self.exec_status == "PAUSED":
                    self.ev("obstacle_detected", safety=self.safety); self.go(S.STOP)
            elif s == S.STOP:
                self.cmd_pub.publish("PAUSE"); self.stop_t = rospy.Time.now(); self.go(S.UPDATE_OBSTACLE_MAP)
                replan_deadline = rospy.Time.now() + rospy.Duration(self.MAX_REPLAN_WAIT)
            elif s == S.UPDATE_OBSTACLE_MAP:
                if (rospy.Time.now() - self.stop_t).to_sec() > self.MAP_SETTLE:
                    self.go(S.REPLAN)
            elif s == S.REPLAN:
                plan = self.plan_to(self.goal)
                if plan is not None and self.safety != "STOP":
                    self.cmd_pub.publish("ABORT")            # discard the old trajectory in Unity
                    rospy.sleep(0.1); self.send(plan)
                    self.cmd_pub.publish("RESUME")           # VERIFY ordering vs your Unity executor
                    self.ev("replanned"); self.go(S.MOVING_TO_TOMATO)
                elif rospy.Time.now() > replan_deadline:
                    self.ev("no_path_timeout"); self.go(S.FAILED_SAFE)
                else:
                    rospy.sleep(0.3)                         # obstacle may leave; try again (Test 6)
            elif s == S.APPROACHING_TOMATO:
                # exclude target, publish margin = approach_margin, Cartesian approach, → GRASPING
                pass
            # GRASPING / HARVESTING / PLACING_TOMATO / RETURNING: your section 1–14 logic,
            # each planned motion re-uses MOVING_TO_TOMATO → STOP → REPLAN.
            elif s == S.FAILED_SAFE:
                self.cmd_pub.publish("PAUSE"); self.ev("safe_failure")
                # wait for operator reset / timeout, then → SEARCHING_FOR_TOMATO
                rospy.sleep(1.0)
            rate.sleep()

    def pregrasp(self, tomato):
        # TODO: pose offset ~0.10 m from tomato along the approach direction, using your grasp orientation
        return tomato.pose

if __name__ == "__main__":
    Harvest().run()
```

Notes: (1) the FSM leaves `PAUSE` held during replanning so the arm does not creep; (2) sending `ABORT` then a new trajectory prevents the old and new trajectories from being played together; (3) after any replan the `MOVING_TO_TOMATO` state resets `exec_status`, so a stale `DONE` is not mis-read.

---

## 24. Static vs Dynamic Obstacle Demonstrations

### 24.1 Demonstration A — Static obstacle

**Scene:** one Box (`ObstacleTag { id = "static_box", isDynamic = false }`) placed between the arm and the target tomato so that the straight tool path intersects it.

**Script:**

1. Start Unity and the ROS stack (`roslaunch harvest_bringup demo.launch`). Confirm the obstacle appears in RViz (`/obstacles/viz`) and in the MoveIt scene.
2. Press start. The FSM detects and localizes the tomato.
3. `CHECKING_PATH` logs `straight_line_fraction < 1.0` (proof the direct path is blocked).
4. `PLANNING_PATH` succeeds with a detour; the trajectory is sent.
5. The arm goes around the box, arrives at the pre-grasp pose, approaches, grasps, harvests, places, returns.

**Expected log:**

```
tomato_localized → straight_line fraction=0.31 → plan_ok (planning_time=…) → moving →
arrived → approaching → grasp_ok → harvest_ok → placed → returned
min_clearance ≥ safety_distance during transit; collisions = 0
```

**Evidence to record:** RViz screenshot of the planned path bending around the box, Unity screen recording, CSV of `min_clearance`, planning time, path length.

### 24.2 Demonstration B — Dynamic obstacle

**Scene:** a `MovingObstacle` (person model or box) that starts outside the workspace and, when triggered, crosses the arm's trajectory.

**Script:**

1. Start the harvest as in A (no obstacle inside the workspace, so the plan is direct).
2. During `MOVING_TO_TOMATO`, trigger the obstacle (press `M`, or publish `dyn_start` on `/unity/scenario_cmd`).
3. `obstacle_detection` reports the obstacle and its velocity; `scene_manager` updates the inflated obstacle in the scene at ≤ 5 Hz.
4. Either `safety_monitor` sees the remaining path becoming invalid (`path_invalid`) or the reactive layer triggers `STOP`; the arm pauses.
5. FSM: `STOP → UPDATE_OBSTACLE_MAP → REPLAN`. While the obstacle is still in the path, `plan_to` fails or returns a longer detour; the FSM waits or replans.
6. When the obstacle leaves (or a detour exists), a new trajectory is sent and the arm continues, reaches the tomato and harvests.

**Expected log:**

```
moving → obstacle_detected (safety_stop, clearance=0.09) → exec PAUSED →
update_map → replan (no_path: waiting) → replanned → moving →
arrived → grasp_ok → harvest_ok → returned
detection_latency_ms=…, stop_latency_ms=…, replan_count=1
```

**Evidence:** timeline plot (arm distance-to-obstacle vs time, with `PAUSE` and `replanned` markers), video, CSV.

Make both demonstrations **repeatable** with a fixed obstacle start position, fixed speed, and a scenario command topic, so results can be compared.

---

## 25. Obstacle Avoidance Testing

| # | Setup (Unity) | Expected behaviour | Pass criteria | Log/measure |
|---|---|---|---|---|
| **1** | No obstacles | Robot reaches tomato normally | Harvest success; collisions = 0; planning time recorded | baseline planning time, path length, harvest time |
| **2** | Static box directly on the shortest path | Collision-free alternative path | `straight_line_fraction < 1`; plan found; collisions = 0; min clearance ≥ `safety_distance − tolerance` | path length vs Test 1, min clearance |
| **3** | Static obstacle beside the arm's route (not blocking) | Robot maintains clearance | min clearance ≥ safety distance for the whole run; near-collision count = 0 | min clearance, near-collision count |
| **4** | Dynamic obstacle enters the path *before* motion starts | Robot plans around it, or waits, **before** moving | No motion until a valid plan exists; `replan_count` = 0 after start; collisions = 0 | planning outcome at start |
| **5** | Dynamic obstacle enters the path *during* motion | Robot pauses and replans | `PAUSE` issued; robot stationary while obstacle within stop distance; then `replanned`; collisions = 0 | detection latency, stop latency, replan count |
| **6** | Obstacle appears in the path, then leaves (`SetActive(false)` or moves away) | Robot resumes and finishes harvesting | Task completes; state trace shows `STOP → REPLAN → MOVING` | time paused, total harvest time |
| **7** | Obstacles arranged so no collision-free path exists (box wall around target) | Robot does **not** force the motion; enters `FAILED_SAFE` and reports | No arm motion into the obstacle; collisions = 0; `safe_failure` event logged | event log |

**Automating tests.** Write a small script that: (a) sets up the scenario via `/unity/scenario_cmd` (dynamic obstacle start/reset) or by loading a Unity scene per test; (b) starts the FSM; (c) waits for a terminal event (`harvest_success` or `safe_failure`) or a timeout; (d) collects metrics from `/harvest/events` and the `/metrics/*` topics into one CSV row. Run each test several times (e.g. 10) and report mean and spread, not a single video.

---

## 26. Obstacle-Avoidance Evaluation Metrics

| Metric | Definition | Measured where | How |
|---|---|---|---|
| **Collision count** | Number of distinct contacts between arm links and obstacles | Unity | `OnCollisionEnter` on arm link colliders, filtered by obstacle layer; publish `/metrics/collisions` |
| **Near-collision count** | Times min clearance dropped below a threshold (e.g. `< 0.5 × safety_distance`) | Unity or ROS | Count threshold crossings of `/metrics/min_clearance` |
| **Minimum obstacle clearance** | Smallest surface-to-surface distance over a run | Unity | `ClearanceProbe` below (approximate), cross-checked with `safety_monitor` link-sphere distances |
| **Path length** | Total end-effector path length (or joint-space length) | ROS | Sum of distances between successive end-effector positions from `tf`, or joint-space distance of the trajectory |
| **Planning time** | Time spent in `plan()` per plan | ROS | Time around `plan()` (or `planning_time` returned in Noetic's tuple; `VERIFY`) |
| **Replanning count** | Number of times `REPLAN` was entered | ROS | Count `replanned` events |
| **Dynamic obstacle detection latency** | Time from ground-truth "obstacle inside the danger zone" to `safety_monitor` flagging it | ROS | Ground-truth crossing event from Unity vs `safety_stop`/`path_invalid` event, both timestamped on the ROS host clock |
| **Robot stopping latency** | Time from `PAUSE` published to Unity reporting `PAUSED` (and, more strictly, to the arm's velocity being zero) | ROS/Unity | `exec_cmd PAUSE` timestamp vs `exec_status PAUSED` timestamp; optionally monitor joint velocity from `/joint_states` |
| **Successful harvesting rate** | Successful harvests / attempts, per test scenario | ROS | Count terminal events over N runs |
| **Average harvesting time** | Mean time from `TOMATO_DETECTED` to `RETURNING` complete | ROS | Timestamps from `state_change` events |

**Why measure latency on the ROS host clock:** the Unity and ROS processes may run on different machines or clocks; timestamping every event when it arrives on the ROS side (`rospy.Time.now()`) gives consistent differences. The TCP delay is included, which is what matters for stopping.

**Unity min-clearance and collision probe (approximate):**

```csharp
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ClearanceProbe : MonoBehaviour
{
    public Transform[] linkPoints;                 // representative points on arm links / tool
    public LayerMask obstacleLayers;
    public float rate = 30f;
    public static int collisions;
    ROSConnection ros; float next;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32Msg>("/metrics/min_clearance");
        ros.RegisterPublisher<Int32Msg>("/metrics/collisions");
    }

    void Update()
    {
        if (Time.time < next) return; next = Time.time + 1f / rate;
        float minD = float.MaxValue;
        foreach (var c in FindObjectsOfType<ObstacleTag>())
        {
            var col = c.GetComponent<Collider>();
            if (col == null) continue;
            foreach (var p in linkPoints)
            {
                // Collider.ClosestPoint is supported for Box/Sphere/Capsule/convex Mesh colliders
                Vector3 cp = col.ClosestPoint(p.position);
                minD = Mathf.Min(minD, Vector3.Distance(cp, p.position));   // point-to-obstacle, not link surface
            }
        }
        ros.Publish("/metrics/min_clearance", new Float32Msg(minD));
        ros.Publish("/metrics/collisions", new Int32Msg(collisions));
    }
}
// On each arm link collider: void OnCollisionEnter(Collision c){ if (((1<<c.gameObject.layer) & mask)!=0) ClearanceProbe.collisions++; }
```

This measures point-to-obstacle distance, not true link-surface distance; subtract each link's approximate radius, or use several points per link. Say so in the report.

**Logging:** record `/harvest/events`, `/metrics/*`, `/obstacles`, `/joint_states`, `/harvest/exec_cmd`, `/harvest/exec_status` with `rosbag record`, plus the CSV summary per run. Keep the Unity random seed / scenario parameters with each run.

---

## 27. Final System Architecture Update

### 27.1 Diagram

```
              ┌──────────────────────────┐
              │          Unity           │
              │  Tomato farm, plants     │
              │  Robot arm (articulated) │
              │  Camera / sensors        │
              │  ObstacleReporter        │
              │  MovingObstacle          │
              │  Trajectory executor     │
              │  + ExecutionGate         │
              │  ClearanceProbe          │
              └────────────┬─────────────┘
                           │  ROS TCP Connection
                           │  (ROS-TCP-Connector ⇄ ROS-TCP-Endpoint)
                           ▼
              ┌────────────────────────────────────────────────────┐
              │                       ROS 1                        │
              │                                                    │
              │  tomato_detection ─→ localization ─→ /tomato_position
              │  obstacle_detection ─→ /obstacles ─→ scene_manager ─┐
              │  robot_state_publisher / TF ◄─ /joint_states       │ │
              │                                                    ▼ │
              │  move_group (MoveIt: planning scene, OMPL, FCL) ◄───┘ │
              │        ▲  plan()                                     │
              │        │                                             │
              │  harvest_fsm ◄── /harvest/safety ◄── safety_monitor  │
              │        │                              ▲             │
              │        └── /harvest/trajectory        └ /obstacles, TF, /check_state_validity
              │  metrics_logger                                     │
              └────────────────────────────┬───────────────────────┘
                                           ▼
                            Collision-free trajectory (RobotTrajectory)
                                           │
                                           ▼
              ┌──────────────────────────────────────────────┐
              │   Robotic arm (Unity)                        │
              │   Avoids obstacles → approaches → harvests   │
              └──────────────────────────────────────────────┘
```

### 27.2 How every subsystem communicates

| Topic / service | Type | Direction | Rate | Purpose |
|---|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Unity → ROS | 10–30 Hz | Tomato detection input |
| `/joint_states` | `sensor_msgs/JointState` | Unity → ROS | 30–50 Hz | Current arm state (planning start state, TF, safety) |
| `/tf` | `tf2_msgs/TFMessage` | `robot_state_publisher` (from URDF + `/joint_states`) | 30–50 Hz | Link and camera frames |
| `/tomato_position` | `geometry_msgs/PoseStamped` | tomato localization → FSM | camera rate | Target pose in `base_link` |
| `/unity/obstacles_gt` | `harvest_msgs/ObstacleArray` | Unity → ROS | 20 Hz | Ground-truth obstacles (Option A) |
| `/obstacles` | `harvest_msgs/ObstacleArray` | `obstacle_detection` → scene_manager, safety_monitor | 20 Hz | Stamped, velocity-annotated obstacle map |
| `/obstacles/viz` | `visualization_msgs/MarkerArray` | `obstacle_detection` → RViz | 20 Hz | Debug visualization |
| `/collision_object` | `moveit_msgs/CollisionObject` | `scene_manager` → `move_group` | ≤ 5 Hz per dynamic obstacle | Planning-scene update |
| `/harvest/safety_distance` | `std_msgs/Float32` | FSM → scene_manager | on change (latched) | Switch transit vs approach margin |
| `/harvest/exclude_obstacle` | `std_msgs/String` | FSM → scene_manager | on event | Keep the target tomato out of the scene |
| `/harvest/trajectory` | `moveit_msgs/RobotTrajectory` | FSM → Unity, safety_monitor | on event | Trajectory to execute and to validate |
| `/harvest/exec_cmd` | `std_msgs/String` | safety_monitor, FSM → Unity | on event | `PAUSE`, `RESUME`, `ABORT` |
| `/harvest/exec_status` | `std_msgs/String` | Unity → FSM | on event | `DONE`, `PAUSED`, `ABORTED` |
| `/harvest/safety` | `std_msgs/String` | safety_monitor → FSM | 20 Hz | `SAFE`, `SLOW`, `STOP`, `PATH_INVALID` |
| `/harvest/events` | `std_msgs/String` (JSON) | all ROS nodes → metrics_logger | on event | Timestamped event log |
| `/unity/scenario_cmd` | `std_msgs/String` | test script → Unity | on event | Start/stop/reset dynamic obstacle |
| `/metrics/min_clearance`, `/metrics/collisions` | `std_msgs/Float32`, `std_msgs/Int32` | Unity → metrics_logger | 30 Hz | Ground-truth metrics |
| `/check_state_validity` | `moveit_msgs/GetStateValidity` (service) | safety_monitor → `move_group` | ~3 Hz | Validate remaining path against scene |
| Gripper command topic | *from sections 1–14* | FSM → Unity | on event | Grasp / release |

### 27.3 Bring-up

```xml
<!-- harvest_bringup/launch/demo.launch  (skeleton; names are placeholders) -->
<launch>
  <rosparam file="$(find harvest_bringup)/config/safety.yaml" command="load" ns="/harvest"/>
  <!-- ROS-TCP-Endpoint per the Unity Hub docs (IP/port from your section 1–14 setup) -->
  <!-- MoveIt: your <robot>_moveit_config move_group.launch + RViz -->
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="rsp"/>
  <node pkg="harvest_bringup" type="obstacle_detection.py" name="obstacle_detection" output="screen"/>
  <node pkg="harvest_bringup" type="scene_manager.py"      name="scene_manager"      output="screen">
    <param name="safety_distance" value="0.10"/>
  </node>
  <node pkg="harvest_bringup" type="safety_monitor.py"     name="safety_monitor"     output="screen"/>
  <node pkg="harvest_bringup" type="harvest_fsm.py"        name="harvest_fsm"        output="screen"/>
  <node pkg="harvest_bringup" type="metrics_logger.py"     name="metrics_logger"/>
</launch>
```

Bring-up order: (1) roscore + ROS-TCP-Endpoint + `move_group` + `robot_state_publisher`; (2) press Play in Unity; (3) confirm `/joint_states`, `/tf` and `/unity/obstacles_gt` are flowing; (4) start the four harvest nodes; (5) check the planning scene in RViz; (6) start the FSM.

---

## 28. Staged Implementation Roadmap

Each stage has an exit criterion. Do not start the next stage until the current one passes it. Every stage is a usable demo on its own.

### Stage 1 — Robot moves to tomato (no obstacles)

- Working: Unity robot ↔ MoveIt via ROS-TCP (as in the Hub pick-and-place approach), `/joint_states` from Unity, tomato pose → pre-grasp plan → executed in Unity.
- **Exit:** Test 1 passes 10/10 with the FSM skeleton (searching → moving → approaching).
- *If stuck:* run the Hub pick-and-place tutorial as-is first, then substitute your arm.

### Stage 2 — Static obstacle avoidance

- Build: `harvest_msgs`, `ObstacleTag`, `ObstacleReporter`, `obstacle_detection`, `scene_manager`.
- First check in RViz that the Unity box appears in the **same place** in the MoveIt scene (frame/axes correct).
- Run `demo_static_detour.py`, then Demonstration A.
- **Exit:** Tests 2, 3 and 7 pass.
- *If stuck:* hard-code one box directly with `PlanningSceneInterface.add_box` (§16.3) to prove planning works, then fix the Unity→ROS link.

### Stage 3 — Dynamic obstacle detection

- Build: `MovingObstacle`, velocity estimation, throttled dynamic scene updates, `safety_monitor` reactive layer, `/obstacles/viz`.
- **Exit:** the moving obstacle appears live in RViz/scene with plausible velocity; safety status flips `SAFE → SLOW → STOP → SAFE` as it crosses (tested with the arm stationary).
- *If stuck:* drop `/check_state_validity` and use only the reactive layer.

### Stage 4 — Dynamic obstacle replanning

- Build: `ExecutionGate` + executor patch, FSM `STOP / UPDATE_OBSTACLE_MAP / REPLAN`, path-validity check.
- **Exit:** Tests 4, 5 and 6 pass; Demonstration B recorded.
- *Fallback if replanning is unreliable:* accept "pause and wait until clear, then replan from the current pose" as the deliverable; this is still valid dynamic handling and is explicitly named in the requirements ("stop, slow down, or replan").

### Stage 5 — Complete harvesting + obstacle avoidance

- Integrate grasp/harvest/place/return with the approach-margin switch (§22.4), attached-object handling, and the metrics logger.
- **Exit:** the full test table (§25) executed N times each; metrics table produced; both demonstrations documented.

**Risk order (most likely to slow you down):** (1) Unity↔ROS coordinate/frame errors (fix by visual check in RViz); (2) trajectory executor patching; (3) tomato-adjacent obstacles making the goal infeasible (§22.4); (4) latency tuning for the safety distance.

---

## Appendix A — Verification checklist for your machine

| Check | Command / action | Expect |
|---|---|---|
| `harvest_msgs` visible to ROS-TCP-Endpoint | `rosmsg show harvest_msgs/Obstacle` | fields as in §19.2 |
| Unity generated C# classes | look under the generated messages folder | `ObstacleMsg`, `ObstacleArrayMsg`; check the namespace |
| Obstacles reach ROS | `rostopic hz /unity/obstacles_gt` | ≈ `publishHz` |
| Frame consistency | RViz: MotionPlanning → Scene Geometry, compare with Unity | boxes coincide |
| Scene accepted by MoveIt | `scene.get_known_object_names()` | contains obstacle ids |
| Collision topic | `rostopic info /collision_object` | `move_group` subscribed |
| Validity service | `rosservice list \| grep validity` | `/check_state_validity` present |
| `plan()` return type | `print(type(group.plan()))` | tuple in Noetic |
| Joint states | `rostopic hz /joint_states` | ≥ 30 Hz |
| Latencies | `/harvest/events` deltas | fill your §22.2 formula |

## Appendix B — Limitations to state in your report

1. Ground-truth obstacle sensing is a simulation shortcut (§17.4).
2. Dynamic handling is *stop → replan → resume*, not continuous reactive avoidance.
3. Constant-velocity prediction only.
4. Primitive over-approximations of leaves, stems and people; over-conservatism can make dense plant regions "unplannable".
5. Instantaneous stopping in Unity articulation drives is optimistic compared to a real arm.
6. Not a safety-rated system and not compliant with industrial robot-safety standards.
7. ROS 1 / MoveIt 1 are end-of-life or maintenance-only; a port to ROS 2 / MoveIt 2 would be needed for production use.
