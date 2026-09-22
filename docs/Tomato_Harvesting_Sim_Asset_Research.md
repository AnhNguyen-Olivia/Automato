# Robotic Arm Tomato Harvesting Simulation — Asset Research Report
**Target stack:** Unity (Windows) + Unity Robotics Hub + ROS 1 (Noetic assumed) + UR10e + OnRobot RG2/RG6

Every link below was pulled from live search results. Where I could not verify a license or a direct download, I say so explicitly rather than guessing. Anything I could not find a real asset for is marked **"No suitable ready-made asset found"** with a primitive-based fallback, per your instructions.

---

## 1. UR10e Robot Model

| Asset | Source | URDF? | Meshes? | ROS 1? | Unity compatible? | License | Recommendation |
|---|---|---|---|---|---|---|---|
| **ur_description (ur10e.urdf.xacro, ur10e_moveit_config)** | github.com/ros-industrial/universal_robot (`kinetic-devel` / `melodic-devel-staging` branches) | ✅ Xacro→URDF | ✅ visual + collision meshes (DAE/STL) | ✅ ROS1 native (Kinetic/Melodic/Noetic) | ✅ Best option | BSD | 🟢 **Primary recommendation** |
| Universal_Robots_ROS_Driver | github.com/UniversalRobots/Universal_Robots_ROS_Driver | Uses `ur_description` above, not its own | — | ✅ ROS1 (official UR driver) | ✅ (via ur_description) | BSD/Apache-2.0 (check repo) | 🟢 Use alongside ur_description if you ever want the real driver architecture, not required for pure simulation |
| Universal_Robots_ROS2_Description | github.com/UniversalRobots/Universal_Robots_ROS2_Description | ✅ Xacro | ✅ | ❌ ROS 2 only | ✅ meshes reusable | BSD-3-Clause | 🟡 Meshes/URDF structure are newer/cleaner, but the package is xacro-templated for `ros2_control`; would need re-writing the transmission/control xacros for ROS1 |
| urdf_files_dataset (Daniella1) | github.com/Daniella1/urdf_files_dataset | ✅ pre-generated flat `ur10e.urdf` | Referenced, not bundled in this repo | N/A (static file) | ✅ (once meshes are sourced) | Mixed/derived — treat as convenience mirror, get meshes from `ros-industrial/universal_robot` | 🟡 Handy if you just want a single flattened URDF file to inspect, but pull the real package for meshes |

**Recommended pick:** `ros-industrial/universal_robot` → `ur_description` package, `ur10e.urdf.xacro`.
Repo: **https://github.com/ros-industrial/universal_robot** (branch `kinetic-devel` is the actively used ROS1 branch; also works on melodic/noetic-devel branches — check the branch selector on GitHub for your ROS1 distro).
- Folder: `ur_description/urdf/ur10e.urdf.xacro` (there's a UR10-e specific xacro plus the shared `ur.urdf.xacro` macro), and `ur10_e_moveit_config/` for MoveIt planning config (confirmed: `ur10_e_moveit_config/config/ur10e.srdf` exists in this repo).
- License: **BSD**, confirmed from the package.xml of the `universal_robots` metapackage in this repo.

**What it includes (verified from the repository structure):**
- ✅ Visual meshes (Collada/DAE) and separate collision meshes (lower-poly STL)
- ✅ All joints (6 revolute joints: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)
- ✅ Joint limits (defined in xacro parameters; note the README explicitly warns that default kinematic parameters are *generic*, not robot-serial-specific — irrelevant for simulation, only matters for real hardware calibration)
- ✅ Inertial data (`cylinder_inertial` macros define mass/inertia per link)
- ✅ Transmission tags (`ur.transmission.xacro`) and Gazebo tags (`ur.gazebo.xacro`) — meaning it is **Gazebo-ready** in addition to being importable
- ✅ End-effector mounting interface: a `tool0`/`ee_link` flange frame is defined at the end of the kinematic chain — this is exactly what you attach the OnRobot gripper to

**Unity URDF Importer compatibility:** Yes. This is a standard, well-formed xacro-based URDF with `<visual>`, `<collision>`, `<inertial>` tags per link — exactly the shape the Unity URDF Importer expects. Workflow:
1. Run `rosrun xacro xacro ur10e.urdf.xacro > ur10e.urdf` (or use the `.xacro` directly with `xacro` on the ROS side) to flatten it into a plain URDF, since Unity's importer does not parse xacro macros/includes itself.
2. Copy the flattened URDF + `meshes/` folder into your Unity `Assets/` folder, preserving relative paths.
3. Right-click the `.urdf` → **Import Robot from Selected URDF file**.
4. Unity will convert the URDF into a hierarchy of `ArticulationBody` components (PhysX 4 articulations), one per link/joint.

---

## 2. OnRobot RG2 Gripper

| Asset | Source | URDF? | Meshes? | ROS 1? | Unity? | License | Recommendation |
|---|---|---|---|---|---|---|---|
| **ur5_rg2_ign** | github.com/AndrejOrsula/ur5_rg2_ign | ✅ standalone RG2 URDF + a combined UR5+RG2 URDF | ✅ STL (collision) + DAE (visual), separated by folder | ❌ built for ROS2/Ignition, but the URDF itself is ROS-version-agnostic XML | ✅ (URDF+meshes import cleanly) | Not stated in the search result — **verify the repo's LICENSE file before use** | 🟡 Best available *pure kinematic* RG2 model; simplifies each finger to a single actuated revolute joint (author's own caveat) |
| **onrobot_description (RG2 + RG6)** | github.com/ikalevatykh/onrobot_ros (mirrored at github.com/libishm1/onrobot1_ros) | ✅ both RG2 and RG6 URDF in one package | ✅ | ❌ ROS2 (colcon/`ros2 launch`) | ✅ (URDF reusable) | Not stated — **verify LICENSE** | 🟡 Only source I found with **both** RG2 and RG6 in the same, consistent package — useful if you want to compare both grippers using identical mesh/joint conventions |
| **OnRobot-RG2FT-ROS** | github.com/ian-chuang/OnRobot-RG2FT-ROS | ✅ | ✅ visual + collision, Gazebo 4-bar-linkage mimic-joint simulation | ✅ **Native ROS1 Noetic**, with a MoveIt config and Gazebo launch files | ✅ | Not stated in result — check repo | 🟢 This is the RG2-**FT** (force-torque) variant, not the plain RG2, but it's geometrically the same 2-finger parallel gripper family and is the only option that is both ROS1-native and Gazebo/MoveIt-ready out of the box. Good structural reference even if you swap in plain-RG2 meshes. |
| onrobot-rg (control library) | github.com/takuya-ki/onrobot-rg | ❌ no URDF/meshes, control-only (Modbus/pymodbus) | ❌ | Python library, ROS-agnostic | N/A | **MIT** (confirmed) | 🟡 Not a 3D asset — useful later if you ever bridge to a *real* RG2/RG6 over Modbus, irrelevant for pure simulation |

**Recommended pick for RG2:** `AndrejOrsula/ur5_rg2_ign` — **https://github.com/AndrejOrsula/ur5_rg2_ign**
- Folder of interest: `ur5_rg2_ign/meshes/{collision,visual}` and `urdf/`
- Contains a standalone RG2 URDF and a combined `ur5_rg2.urdf` you can use as a wiring reference for how the author attached RG2 to a UR5 flange (the same procedure applies to UR10e, since UR e-series share the same ISO 50mm tool-flange standard).

**Attaching RG2 to the UR10e flange — is extra work required? Yes, some:**
1. The UR10e URDF ends in a `tool0` (or `ee_link`) frame.
2. The RG2 URDF's base link needs a `<joint type="fixed">` connecting it to `tool0`, with the correct z-offset/rotation matching OnRobot's real mounting depth (OnRobot grippers mount via the URCap Quick Changer on a UR flange — for simulation you can approximate this as a simple fixed offset of a few centimeters along the flange's Z axis; exact numeric offsets are in OnRobot's own CAD/mounting drawings, not present in the community URDFs above, so you'll want to sanity-check the offset visually in Unity/RViz rather than trust a hardcoded number).
3. Because the RG2 URDF was authored independently of `ros-industrial/universal_robot`'s UR10e, **link/joint naming will very likely collide or mismatch** (e.g., both files might define a `base_link`). You'll need to either: (a) merge both xacros into one top-level xacro file with a `prefix` argument (the standard ROS-Industrial pattern, which `ur_description` already supports via `xacro:macro ... params="prefix"`), or (b) import both URDFs into Unity separately and manually parent the gripper's root `ArticulationBody`/Transform under the UR10e's `tool0` GameObject after import. Option (b) is faster for a university project; option (a) is the "correct" ROS-native way and lets Gazebo/MoveIt see a single combined robot.

---

## 3. OnRobot RG6 Gripper + RG2 vs RG6 Comparison

**Recommended pick for RG6:** `ikalevatykh/onrobot_ros` `onrobot_description` package — **https://github.com/ikalevatykh/onrobot_ros** — contains both `RG2` and `RG6` URDFs side by side, described as being for interfacing with UR3/UR5/UR10 from ROS2.

| Criterion | RG2 | RG6 |
|---|---|---|
| ROS/URDF asset availability | Slightly better — appears in 2 independent repos (`AndrejOrsula/ur5_rg2_ign`, `ikalevatykh/onrobot_ros`) plus the RG2-FT ROS1 package as a structural analog | Appears in 1 repo I could verify (`ikalevatykh/onrobot_ros`) |
| Unity-compatible models | Same — both are plain URDF+mesh, import identically | Same |
| Ease of simulation | Slightly easier — more prior art (Ignition/Gazebo examples exist for RG2) | Slightly harder — less community reference, same mesh/joint structure though |
| Gripper complexity | Smaller stroke (max ~110mm open, ~5.5kg payload real hardware) → simpler, shorter finger travel to simulate | Larger stroke (max ~160mm open, ~6kg payload) → wider open jaw, arguably *easier* to guarantee a tomato of unknown size fits between the fingers without fine-tuned closing distance |
| Suitability for grabbing a simulated tomato | Good if your tomato model is small/medium (typical tomato ≈ 5–7cm diameter — well within RG2's range) | Also good, with more margin for larger/beefsteak tomato models or imprecise localization (a few cm of position error still lets the wider jaw close around the fruit) |
| Documentation | Same OnRobot official spec sheets apply to both (product pages, not simulation docs) | Same |
| License | Depends on which repo — verify LICENSE file per repo, not confirmed here | Same caveat |
| Integration effort with UR10e | Same procedure/pitfalls as described in Section 2 | Same procedure/pitfalls |

**Practical recommendation for this project: RG2.** It has marginally more independent community URDF sources (reducing single-point-of-failure risk if one repo is abandoned/broken), and its jaw stroke is a closer match to a single tomato's diameter, which will make your grasp-detection logic (e.g., "did the fingers close around something within X cm?") more physically meaningful for a harvesting demo. If your tomato models turn out to be larger cluster/beefsteak varieties, switch to RG6 — the same `onrobot_description` repo has both, so switching is a matter of pointing the URDF importer at a different file, not a new integration effort.

---

## 4 & 5. Tomatoes and Tomato Plants

### Single tomato fruit (free / attribution)

| Asset | Source & link | Format | License | Textures | Commercial/academic use | Poly count | Unity import |
|---|---|---|---|---|---|---|---|
| **"Tomato" by rhcreations** | sketchfab.com/3d-models/tomato-a64e1ac7f8c44821bbc365f0581d95b2 | Downloadable via Sketchfab (glTF/FBX/OBJ auto-conversion) | **CC Attribution (CC-BY)** — confirmed on page | Yes (Cycles-rendered material, textures included in download) | ✅ commercial + academic OK, **attribution required** (credit rhcreations) | 1.3k tris / 689 verts — light enough for real-time sim | ✅ Sketchfab's Unity importer plugin, or download glTF and import directly |
| **"プチトマト" (cherry tomato) by kukumoto** | sketchfab.com/3d-models/c684983f26b74e7291737871d00643fb | Sketchfab download | **CC Attribution** | Unstated — check on download | ✅ with attribution | 480 tris / 248 verts — extremely lightweight, good for scattering many instances | ✅ |
| **"tomato" by dayoung.univ** (Tilt Brush sculpt) | sketchfab.com/3d-models/tomato-bfb29a43f284428c9be5b1d0673585bc | Sketchfab download | **CC Attribution-ShareAlike** | Stylized, hand-painted (Tilt Brush) look, not photoreal | ✅ commercial with attribution, **ShareAlike** means derivatives must carry the same license | 8.7k tris | ✅ |
| **"Tomatoes" (photogrammetry) by John Toeppen** | sketchfab.com/3d-models/tomatoes-a5ce45774a184f4e9647538b37203d5a | Sketchfab | **License not fully confirmed on the excerpt — page shows a "NoAI" tag (blocks generative-AI training use) but not the reuse license itself; check the license badge on the page before use** | Photogrammetry-captured (realistic) | ⚠️ verify before commercial/academic use | 36k tris (clean this down for real-time use) | ✅ once license confirmed |

Note: Sketchfab requires a free account to download models (not a paid subscription) — this satisfies your "no paid subscription" requirement but does need a login.

### Single tomato fruit (paid, higher fidelity — for reference/comparison only)
- **CGTrader "Tomato 1"** — https://www.cgtrader.com/3d-models/food/vegetable/tomato-1-97f1e73f-3939-4dbb-9694-82696fa30cf0 — $3, Royalty Free license, OBJ/FBX/MAX/MA, high-poly + low-poly (3,484 faces) versions with 8K color+bump textures. Good if free CC-BY tomatoes aren't visually convincing enough.
- **Sketchfab "Vegetable Tomato - Photoscanned PBR"** — https://sketchfab.com/3d-models/vegetable-tomato-photoscanned-pbr-51f230b89b144f20bd3c30b0002b9c0a — photogrammetry-based, 3.1k triangles, PBR textures (diffuse/bump/specular/AO at 4096²), explicitly marketed as **Unity3D-ready** with a `.unitypackage` export option. Price/license tier not shown in the excerpt I retrieved — check the page for current price and license terms before buying.
- **RenderHub "Tomato Low Poly" / "Tomato Low Poly 2"** by frezzy — https://www.renderhub.com/frezzy/tomato-low-poly and .../tomato-low-poly-2 — Extended Use License (commercial + non-commercial allowed per their terms), ~1,700–1,800 tris, OBJ/FBX, textured/PBR.

### Tomato plant (with tomatoes attached, stem/leaves/branches)

| Asset | Source | Tier | Format | License | Notes |
|---|---|---|---|---|---|
| **"Vegetable Plants" asset pack** (dexsoft-games) | assetstore.unity.com/packages/3d/vegetation/plants/vegetable-plants-update-2394 | **Paid** (Unity Asset Store standard EULA) | Unity prefabs (native, no import step) | Unity Asset Store EULA — commercial use allowed under standard Unity asset license | Confirmed to include a dedicated **"Tomato Plant"** prefab (6,327 tris) and a separate **"Tomato Fruit"** prefab (1,372 tris), each with up to 3 LOD stages — directly matches your "tomato attached to plant" + "tomato fruit" + "multiple varieties" requirements in one purchase |
| **"Low Poly Garden Pack"** | assetstore.unity.com/packages/3d/vegetation/plants/low-poly-garden-pack-68789 | **Paid** ($10) | Unity prefabs | Unity Asset Store EULA | Confirmed to include a **"Tomato Plant"** among other garden vegetables, low-poly/mobile-optimized, diffuse+normal(+metallic) maps |
| **"Greenhouse Low Poly Plants"** | assetstore.unity.com/packages/3d/vegetation/plants/greenhouse-low-poly-plants-174487 | **Paid** | Unity prefabs | Unity Asset Store EULA | Stylized fantasy pack that includes a **"Blood berry (tomato)"** plant in 2 growth variants — visually stylized, not photoreal; only use this if visual realism doesn't matter to you (you already said it doesn't) |
| **Tomato Plant 3D Model** (mehrazvira) | superhivemarket.com/products/tomato-plant-3d-model (mirrored on Blender Market and Gumroad) | **Paid** (~$5–7) | Blender-native, exports to FBX/OBJ/Unreal/Unity-compatible formats | "Royalty Free" per listing | Most complete free-standing tomato plant model I found: **3 growth stages** (seedling / fruiting / mature) in one package, each stage modeled with stem+leaves+fruit, real-world scaled (116–230cm) |
| Full CC0/free complete plant model | — | — | — | — | **No suitable ready-made *free* complete tomato-plant asset (stem+branches+leaves+fruit as one rigged/organized model) was found.** Every complete-plant asset I could verify is paid. |

**If you need a free complete plant:** build a simple procedural stand-in instead of buying an asset:
- Trunk/stem: a tapered `Cylinder` or a Unity `LineRenderer`/simple mesh spline.
- Branches: 2–4 child cylinders angled outward.
- Leaves: flat quads or low-poly leaf meshes (many single free leaf models exist on Sketchfab under CC-BY, e.g. search "leaf" filtered to downloadable) with a simple green material, instanced along the branches.
- Fruit: attach the free CC-BY tomato models above (Section 4) at the leaf axils.
- This is *more* work upfront but gives you full control over where tomatoes are positioned (important, since your pipeline needs known-good 3D ground-truth positions for evaluating your detection/localization accuracy — a paid pre-made plant may bury the tomatoes inside dense foliage in ways that are hard to control for your vision pipeline).

### Academic / research datasets (for reference, not always usable as 3D Unity meshes)
I did not find a specific, verifiably-downloadable academic 3D tomato-plant *mesh* dataset in this pass (agricultural robotics vision papers typically release RGB-D **image datasets**, not textured 3D CAD/mesh plant models, e.g. the "Tomato Detection Dataset" style releases used in fruit-detection papers). If your report needs an academic citation for realism, cite such an image dataset for validating your detector, but continue using the Sketchfab/Asset Store meshes above for the actual Unity 3D scene, since image datasets are not meshes and can't be dropped into Unity.

---

## 6. Static Obstacles

**Recommendation: build almost all of these directly in Unity, do not download assets.**

| Obstacle | Best approach | Why |
|---|---|---|
| Boxes / crates | Unity `Cube` primitive, scaled, + `BoxCollider` | Free, perfect box collider, zero import/config work |
| Poles / support beams | Unity `Cylinder` primitive + `CapsuleCollider` or `MeshCollider` (convex) | Same reasoning |
| Walls / partitions | Unity `Cube` (flattened) or Unity's built-in `ProBuilder` package for L-shaped walls | Trivial colliders, no texture needed for a robotics demo |
| Containers / collection bins (your harvesting drop-off point) | Unity `Cube` array (open-top box made of 5 cubes) | You need this to be functionally a container with a real collider — primitives guarantee that; a downloaded crate mesh often has a non-convex collider that needs manual fixing |
| Farm equipment / greenhouse structure (visual only, not something the arm interacts with) | *Optional* downloaded asset for visual context, e.g. Unity Asset Store greenhouse/farm packs (paid) or free single-prop models from Sketchfab (CC-BY, verify each) | Only worth the effort if you want scene "dressing"; use a simple `BoxCollider` bounding volume around any such prop rather than its detailed mesh collider, for cheap and reliable collision checks |

Use Unity primitives specifically because: (1) their colliders are analytic (box/sphere/capsule), which is far cheaper for real-time collision checks than a downloaded mesh's convex/concave collider, (2) zero import/licensing overhead, (3) your grading criteria (per your own priority list) rank "visual quality" last — so time is better spent on the path-planning/collision logic than on obstacle art.

---

## 7. Dynamic Obstacles

| Option | Source | Free? | Recommendation |
|---|---|---|---|
| **Capsule / simple primitive "person"** | Built into Unity | ✅ Free | 🟢 **Recommended for this project.** A `Capsule` (or a `Capsule` + small `Sphere` "head") moving along a scripted or `NavMeshAgent` path through the robot's workspace gives you a controllable, cheap-to-collide-with proxy. It's exactly what most academic robotics-simulation papers use for "human obstacle" demonstrations — a rigged photorealistic human is not necessary to demonstrate detect→stop→replan behavior, and a capsule collider is trivial to reason about in your obstacle-avoidance code. |
| Rigged humanoid model (visual realism) | Adobe **Mixamo** (mixamo.com) — free, no-account-needed-per-model rigged human characters with walk-cycle animations, exportable as FBX | ✅ Free (Adobe account required, not a paid subscription) | 🟡 Use *only* if your instructor/rubric wants a visually convincing human — attach a simple `CapsuleCollider` to the rigged mesh regardless, so your collision-avoidance logic still only has to reason about a capsule, not the animated mesh surface |
| Moving crate / agricultural cart | Unity `Cube`/`Cylinder`(wheels) primitive on a scripted translate or `NavMeshAgent` path | ✅ Free | 🟢 Simple and sufficient for a "moving box" dynamic obstacle demonstration |
| Another robot (dynamic obstacle) | Reuse a second, simplified URDF import (e.g. a second small arm, or just a moving primitive standing in for a mobile robot/AGV) | Depends on which URDF you reuse | 🟡 Only pursue this if your rubric specifically wants a robot-vs-robot avoidance demo; otherwise it adds import/licensing overhead for no functional benefit over a primitive |

**Bottom line:** use a `Capsule` primitive (optionally re-skinned with a free Mixamo character for visuals) driven by a `NavMeshAgent` or a scripted waypoint path that crosses the UR10e's workspace. This satisfies "detection → stop/replan → avoidance" without needing licensing review on a downloaded human model.

---

## 8. Camera / Sensor Simulation — **Updated: Intel RealSense D435**

You've selected the **Intel RealSense D435** (RGB + depth in one unit). Good choice for this project — it's the single most common sensor in real tomato-harvesting robotics papers, and unlike the generic options below, it has an **official, ROS1-supported, Apache-2.0-licensed description package** straight from Intel.

| Asset | Source | Contains | ROS 1? | License | Recommendation |
|---|---|---|---|---|---|
| **realsense2_description** (D435 URDF/xacro + meshes) | github.com/IntelRealSense/realsense-ros → `realsense2_description/urdf/_d435.urdf.xacro` | Full URDF: visual mesh (correct D435 housing geometry), `color`/`depth`/`infrared1`/`infrared2` optical frames with **real, correct extrinsic offsets** between the RGB and depth sensors (the actual physical baseline on the real D435), joints, inertials | ✅ **Official ROS1 support confirmed** — release notes explicitly list Noetic, Melodic, and Kinetic as supported distros | **Apache License 2.0** (confirmed from package.xml) | 🟢 **Primary recommendation.** This is the real Intel-maintained package, not a community mirror — use it as your D435 mounting/geometry asset. |
| realsense2_description mirror w/ Gazebo view launch | github.com/issaiass/realsense2_description (+ companion github.com/issaiass/realsense_gazebo_plugin) | Same `realsense2_description` folder, packaged with a ready `roslaunch realsense2_description view_d435_model_rviz_gazebo.launch` and a Gazebo depth/RGB plugin | ✅ ROS1 (Kinetic/Melodic/Noetic-era) | Inherits Apache 2.0 from upstream `realsense2_description`; verify the plugin repo's own LICENSE | 🟡 Useful if you also want a Gazebo-side preview/sanity-check of the D435 mount before wiring it into Unity, not required for the Unity pipeline itself |

**How this fits your Unity + ROS1 pipeline:**
1. **Import `realsense2_description`'s D435 URDF/meshes into Unity** the same way you import the UR10e and gripper — right-click → *Import Robot from Selected URDF file*. This gives you the correct D435 housing mesh and, critically, the correct `color_optical_frame` / `depth_optical_frame` transform offsets already baked in (so your published depth data and RGB data stay properly aligned, exactly as they would on the real device).
2. Parent the imported D435 `GameObject` to wherever you're mounting it (e.g., a fixed post above the tomato plants, or the UR10e's wrist for an eye-in-hand setup) via a fixed joint/transform.
3. **RGB stream:** attach a Unity `Camera` at the `color_optical_frame` child transform, render to a `RenderTexture`, publish as `sensor_msgs/Image` over **ROS-TCP-Connector** — matching real D435 RGB resolution/FOV (1920×1080 max, commonly run at 640×480 or 1280×720; horizontal FOV ≈69°, vertical ≈42° at 2m per Intel's spec).
4. **Depth stream:** attach a second Unity `Camera` (or reuse the same one with `Camera.depthTextureMode = DepthTextureMode.Depth`) at the `depth_optical_frame` transform, convert the depth buffer to a `sensor_msgs/Image` (32FC1) or `sensor_msgs/PointCloud2`, and publish it on the equivalent of the real driver's `/camera/depth/image_rect_raw` topic name if you want your ROS-side code to be a drop-in match for the real `realsense-ros` driver's topic conventions.
5. There is **no need to simulate the real D435's IR-stereo noise/dropout pattern** — Unity's exact per-pixel depth buffer is strictly better (noise-free) ground truth, which is appropriate for a university-scope sim where the priority is correct localization logic, not sensor-noise robustness research.
6. **LiDAR** (optional, not part of your D435 choice): if you still want a distinct obstacle-detection sensor separate from the D435, a simple `Physics.RaycastAll` sweep script publishing `sensor_msgs/LaserScan` remains the easiest DIY option — no ready-made Unity LiDAR package exists in Robotics Hub. Not necessary if you're using the D435's own depth stream for both tomato localization and obstacle detection.

**Recommendation:** Import the **official `realsense2_description` D435 URDF+meshes** (Apache-2.0, ROS1-supported) for correct geometry/mounting, then drive its RGB and depth output using two Unity `Camera` components (color + depth texture) published through **ROS-TCP-Connector**, following the same camera-publishing pattern Unity's own **Robotics Object Pose Estimation** demo already uses. This gives you a visually and kinematically accurate D435 stand-in without needing to hand-author sensor geometry or emulate IR-stereo hardware.

---

## 9. Complete Asset List

| Component | Asset | Source | Format | ROS/URDF | Unity | License | Cost | Required? |
|---|---|---|---|---|---|---|---|---|
| Robot | UR10e (`ur_description`) | github.com/ros-industrial/universal_robot | URDF/Xacro + DAE/STL | ✅ ROS1 | ✅ URDF Importer | BSD | Free | YES |
| Gripper | RG2 | github.com/AndrejOrsula/ur5_rg2_ign | URDF + STL/DAE | ROS2-authored, URDF reusable in ROS1 | ✅ | Verify repo LICENSE | Free | YES |
| Gripper (alt.) | RG2 + RG6 | github.com/ikalevatykh/onrobot_ros | URDF + meshes | ROS2-authored, URDF reusable | ✅ | Verify repo LICENSE | Free | Optional (comparison) |
| Tomato fruit | "Tomato" by rhcreations | sketchfab.com/3d-models/tomato-a64e1ac7f8c44821bbc365f0581d95b2 | glTF/FBX/OBJ (via Sketchfab export) | N/A | ✅ | CC-BY (attribution) | Free | YES |
| Tomato plant | "Vegetable Plants" pack | assetstore.unity.com/packages/3d/vegetation/plants/vegetable-plants-update-2394 | Unity prefab | N/A | ✅ native | Unity Asset Store EULA | Paid | YES |
| Static obstacle | Unity primitives (Cube/Cylinder) | Built into Unity | Native GameObject | N/A | ✅ native | N/A | Free | YES |
| Dynamic obstacle | Unity `Capsule` (+ optional Mixamo skin) | Built into Unity / mixamo.com | Native / FBX | N/A | ✅ | N/A / Adobe Mixamo terms | Free | YES |
| Camera/sensor | Intel RealSense D435 (`realsense2_description` URDF/meshes) + Unity `Camera` (color+depth) | github.com/IntelRealSense/realsense-ros (`realsense2_description`) | URDF/Xacro + meshes | ✅ ROS1 (Noetic/Melodic/Kinetic) | ✅ URDF Importer | **Apache-2.0 (confirmed)** | Free | YES |
| Perception/RGB capture support | Unity Perception package | github.com/Unity-Technologies/com.unity.perception | Unity package | Publishes via ROS-TCP-Connector | ✅ native | Apache-2.0 (verify current release) | Free | Optional (labeling/ground truth) |
| ROS bridge | ROS-TCP-Connector / ROS-TCP-Endpoint | github.com/Unity-Technologies/Unity-Robotics-Hub (component repos) | Unity package + ROS node | ✅ ROS1 supported | ✅ | Apache-2.0 | Free | YES |
| URDF import tool | URDF-Importer | github.com/Unity-Technologies/URDF-Importer | Unity package (Git URL) | N/A | ✅ | **Apache-2.0** (confirmed) | Free | YES |

---

## 10–11. Exact Download Links & License Summary

| # | Asset | Exact link | License | Attribution required? | Commercial use? | Notes |
|---|---|---|---|---|---|---|
| 1 | UR10e URDF | https://github.com/ros-industrial/universal_robot | BSD | No | Yes | Check `kinetic-devel`/`melodic-devel-staging`/`noetic-devel` branch matching your ROS1 distro |
| 2 | Unity URDF Importer | https://github.com/Unity-Technologies/URDF-Importer (Package Manager Git URL: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer#v0.5.2`) | Apache-2.0 | No | Yes | Confirmed license file present in repo |
| 3 | Unity Robotics Hub (tutorials, TCP Connector/Endpoint, Pose Estimation demo) | https://github.com/Unity-Technologies/Unity-Robotics-Hub and https://github.com/Unity-Technologies/Robotics-Object-Pose-Estimation | Apache-2.0 (standard for Unity-Technologies robotics repos — verify per-repo LICENSE file) | No | Yes | The Pose Estimation repo is your best architectural template: UR3 + Robotiq gripper + camera + Perception package + ROS1 Noetic, end to end |
| 4 | OnRobot RG2 URDF | https://github.com/AndrejOrsula/ur5_rg2_ign | **Unconfirmed — check repo LICENSE file** | Unconfirmed | Unconfirmed | Flag: verify before submitting for grading if your institution requires cleared licenses |
| 5 | OnRobot RG2 + RG6 URDF | https://github.com/ikalevatykh/onrobot_ros | **Unconfirmed — check repo LICENSE file** | Unconfirmed | Unconfirmed | Same flag as above |
| 6 | OnRobot RG2-FT ROS1 package (structural reference) | https://github.com/ian-chuang/OnRobot-RG2FT-ROS | **Unconfirmed — check repo LICENSE file** | Unconfirmed | Unconfirmed | Same flag |
| 7 | OnRobot RG control library (Modbus, no meshes) | https://github.com/takuya-ki/onrobot-rg | **MIT (confirmed)** | No | Yes | Control-only, not a 3D asset |
| 8 | Tomato fruit (rhcreations) | https://sketchfab.com/3d-models/tomato-a64e1ac7f8c44821bbc365f0581d95b2 | CC-BY | **Yes** | Yes | |
| 9 | Cherry tomato (kukumoto) | https://sketchfab.com/3d-models/c684983f26b74e7291737871d00643fb | CC-BY | **Yes** | Yes | |
| 10 | Tomato plant pack | https://assetstore.unity.com/packages/3d/vegetation/plants/vegetable-plants-update-2394 | Unity Asset Store EULA | No (standard EULA, not CC) | Yes | Paid |
| 11 | Low Poly Garden Pack (has tomato plant) | https://assetstore.unity.com/packages/3d/vegetation/plants/low-poly-garden-pack-68789 | Unity Asset Store EULA | No | Yes | Paid, $10 |
| 12 | Perception package | https://github.com/Unity-Technologies/com.unity.perception | Apache-2.0 (verify per current release) | No | Yes | Official Unity Computer Vision |
| 13 | Mixamo (optional human skin for dynamic obstacle) | https://www.mixamo.com | Adobe General Terms of Use (free for use in projects; not CC) | No | Yes for most uses — read Adobe's terms if publishing the project itself | Free, Adobe-account gated, not a paid subscription |
| 14 | **Intel RealSense D435 URDF + meshes (`realsense2_description`)** | https://github.com/IntelRealSense/realsense-ros | **Apache License 2.0 (confirmed)** | No | Yes | Official Intel package; ROS1 Noetic/Melodic/Kinetic support explicitly listed in release notes; file of interest: `realsense2_description/urdf/_d435.urdf.xacro` |
| 15 | D435 Gazebo/RViz preview mirror (optional) | https://github.com/issaiass/realsense2_description + https://github.com/issaiass/realsense_gazebo_plugin | Inherits Apache-2.0 from upstream description; **verify the plugin repo's own LICENSE separately** | No | Likely yes, unconfirmed for the plugin repo | Convenience mirror with a ready `roslaunch ... view_d435_model_rviz_gazebo.launch`; only needed if you want a Gazebo sanity-check before Unity import |

**Flag:** entries 4, 5, 6 have licenses I could not confirm from the search excerpts. Open each repository's `LICENSE` file directly before relying on them for a graded/publishable project. None of them show signs of being pirated/ripped — they are original community CAD/URDF work — but "no stated license" is not the same as "cleared for reuse," so verify.

---

## 12. ROS 1 Compatibility Notes

- **UR10e (`ros-industrial/universal_robot`)** — native ROS1, no conversion needed.
- **RG2 (`AndrejOrsula/ur5_rg2_ign`)** and **RG2/RG6 (`ikalevatykh/onrobot_ros`)** — authored for ROS2 (colcon build, `ros2 launch` files). **The URDF/xacro and mesh files themselves are not ROS-version-specific** — URDF is just XML understood identically by ROS1 and ROS2 tooling. What you must change:
  - Ignore/delete the `CMakeLists.txt`/`package.xml` `ament_cmake` build boilerplate; replace with a standard ROS1 `catkin` `package.xml` (format 1 or 2) and `CMakeLists.txt` if you want it as a proper catkin package (optional — for pure Unity import you don't need a ROS package at all, just the raw `.urdf`/`.xacro` + `meshes/` folder).
  - Any `ros2_control`/`<ros2_control>` tags are unused in ROS1 and can be stripped or ignored (Unity's importer ignores control-plugin tags anyway; it only reads `<link>`, `<joint>`, `<visual>`, `<collision>`, `<inertial>`).
  - Gazebo-Ignition-specific plugin tags (if targeting classic Gazebo instead of Ignition) would need updating, but this is irrelevant to your Unity-only pipeline.
- **OnRobot-RG2FT-ROS** — already native ROS1 Noetic, no conversion needed, only relevant as a *structural template* (its Gazebo/MoveIt config) since it's the FT variant not plain RG2/RG6.
- **Intel RealSense D435 (`realsense2_description`)** — native ROS1, no conversion needed; Intel's own release notes explicitly confirm Noetic/Melodic/Kinetic support. No xacro/tag stripping required, unlike the ROS2-authored gripper repos.

---

## 13. Unity Compatibility Notes

- **URDF Importer** (`Unity-Technologies/URDF-Importer`) parses standard URDF, builds a hierarchy of `ArticulationBody` components (Unity's PhysX 4 articulation solver — not regular `Rigidbody`+`Joint`), which is required for stable, accurate multi-DOF arm simulation. Confirmed limitation from the repo's issue tracker: articulation chains are capped around 64 joints per hierarchy — irrelevant for a single UR10e (6 DOF) + gripper (1–2 actuated DOF).
- **Joint limits** — read directly from each `<joint><limit lower=... upper=.../>` tag and applied to the corresponding `ArticulationBody`'s drive limits.
- **Collision meshes** — the importer offers a mesh-decomposition setting at import time (convex decomposition options) since PhysX articulation colliders generally need convex shapes; the UR10e package's dedicated lower-poly collision meshes make this fast and accurate.
- **Coordinate systems** — URDF is right-handed, Z-up; Unity is left-handed, Y-up. The importer handles this conversion automatically during import (this is one of the main reasons to use the importer rather than hand-porting meshes).
- **Materials/textures** — DAE/Collada visual meshes typically carry their own material references; the importer creates corresponding Unity materials, though colors/shaders often need minor cleanup for URP/HDRP if your project uses one of those render pipelines instead of Built-in RP.

---

## 14. Recommended Unity Project Structure

```
Assets/
├── Robots/
│   ├── UR10e/                  # imported URDF + meshes from ros-industrial/universal_robot
│   └── OnRobot/
│       ├── RG2/                # imported URDF + meshes
│       └── RG6/                # imported URDF + meshes (optional, for comparison)
│
├── Environment/
│   ├── TomatoPlants/            # purchased/plant pack prefabs
│   ├── Tomatoes/                # individual CC-BY tomato meshes
│   ├── StaticObstacles/         # primitive-based prefabs (crates, poles, bins)
│   └── DynamicObstacles/        # capsule/Mixamo prefab + waypoint scripts
│
├── Sensors/
│   └── Camera/                  # Perception Camera prefab, depth-capture script
│
├── ROS/
│   ├── Messages/                 # generated C# message classes (Image, PointCloud2, etc.)
│   └── Settings/                 # ROS-TCP-Connector settings asset (ROS IP, port)
│
├── Scripts/
│   ├── Detection/                 # tomato-detection interface / model inference wrapper
│   ├── Localization/              # depth→3D position conversion
│   ├── Planning/                   # path-planning glue code (MoveIt service calls)
│   ├── ObstacleAvoidance/          # dynamic obstacle detection + replanning triggers
│   └── Gripper/                    # gripper open/close control + grasp-success check
│
└── Scenes/
    └── TomatoHarvestingDemo.unity
```

This mirrors the structure you proposed; the only addition is splitting `ROS/` into `Messages/`+`Settings/` and `Scripts/` into functional subfolders matching your 14-step pipeline, since a flat `Scripts/` folder gets unwieldy once you have detection, localization, planning, and gripper logic all mixed together.

---

## 15. Integration Difficulty

| Asset | Difficulty | Why |
|---|---|---|
| UR10e URDF (`ros-industrial/universal_robot`) | 🟢 Easy | Standard, well-formed, widely-used package; direct match for Unity's URDF Importer expectations |
| Unity URDF Importer + ROS-TCP-Connector setup | 🟢 Easy | Official Unity packages with first-party documentation and the Pose Estimation demo as a working reference |
| RG2/RG6 URDF (community repos) | 🟡 Moderate | Correct URDF structure, but authored for ROS2 tooling and a different robot (UR5, or standalone) — you must merge/re-parent it onto the UR10e's `tool0` frame yourself, and mounting offsets aren't guaranteed accurate for e-series |
| Combining UR10e + RG2/RG6 into one kinematic chain | 🟡 Moderate | Requires either xacro merging (ROS-native, more correct) or manual GameObject re-parenting in Unity after separate imports (faster, less "correct" but fine for a demo) |
| Tomato fruit (CC-BY Sketchfab meshes) | 🟢 Easy | Small, static meshes — drop into Unity, add `SphereCollider`, done |
| Tomato plant (paid Asset Store pack) | 🟢 Easy | Native Unity prefabs, no conversion needed, but costs money |
| Free complete tomato plant | 🔴 Difficult (if attempted) | No good free complete-plant asset exists; building one procedurally from leaf+stem+fruit primitives is real modeling work, not a quick import |
| Static obstacles (primitives) | 🟢 Easy | Built into Unity, zero import step |
| Dynamic obstacle (capsule) | 🟢 Easy | Built into Unity; scripting the movement path is the only work, and that's inherent to your project regardless of asset source |
| Dynamic obstacle (Mixamo-skinned) | 🟡 Moderate | Requires rigging/retargeting the animation to your capsule's collider timing, and Adobe account/export step |
| Intel RealSense D435 URDF/meshes (`realsense2_description`) | 🟢 Easy | Official Intel package, Apache-2.0, native ROS1 support, correct housing mesh + real color/depth extrinsic offsets already defined — imports like any other URDF |
| D435 RGB stream (Unity Camera → ROS Image) | 🟢 Easy | Official Unity workflow (Camera → ROS-TCP-Connector), documented, used in the Pose Estimation demo |
| D435 depth stream → 3D tomato localization | 🟡 Moderate | No off-the-shelf "depth-to-ROS" package ships by default; you write a short C# script to read `Camera.depthTexture` at the D435's `depth_optical_frame` and pack/publish it — not hard, but not zero-code either |
| A random unverified Sketchfab gripper mesh with no joints/URDF (hypothetical, avoid) | 🔴 Difficult | No kinematics, no collision structure — would need full manual rigging in Unity, defeats the purpose of using ROS/URDF at all |

---

## 16. Missing / No-Asset-Found Items

- **No suitable ready-made *free*, complete (stem+branches+leaves+fruit, organized/rigged) tomato plant model was found.** → Build procedurally: tapered cylinder stem, angled cylinder branches, low-poly leaf quads/meshes (source individual free CC-BY leaf models), with the free CC-BY tomato fruit meshes (Section 4) attached at branch junctions. This gives you full control over tomato placement, which your localization pipeline needs anyway.
- **No verified free/CC0 OnRobot RG2 or RG6 model with confirmed license was found** (the URDF/meshes exist and look structurally sound, but their licenses aren't stated in what I could retrieve). → Not a primitive-replaceable item (you need real kinematics for the gripper), so the action item is: open the two repos' LICENSE files yourself and confirm before relying on them, rather than substituting a primitive.
- **No dedicated free "RealSense simulator" Unity package was found.** → Use Unity's own `Camera.depthTexture` (exact ground-truth depth) instead of trying to simulate RealSense-specific IR/stereo noise — simpler, and sufficient for your stated priorities.
- **No free LiDAR-in-Unity package was found bundled with Robotics Hub.** → If you decide you need LiDAR at all (optional per Section 8), implement it as a simple `Physics.RaycastAll` sweep script; this is a common, well-documented DIY pattern, not something you need to download.

---

## 17. Final Recommended Asset Set

**Robot:** UR10e via `ur_description` in **https://github.com/ros-industrial/universal_robot** (BSD license, ROS1-native, includes visual+collision meshes, joints, joint limits, inertials, and a flange mount frame).

**Gripper:** OnRobot **RG2** via **https://github.com/AndrejOrsula/ur5_rg2_ign** (URDF+meshes; verify LICENSE file; re-parent onto the UR10e's `tool0` frame manually in Unity, or merge xacros if you want a single ROS-native combined description). RG6 is available as a drop-in alternative from **https://github.com/ikalevatykh/onrobot_ros** if your tomato models turn out larger than expected.

**Tomato:** "Tomato" by rhcreations — **https://sketchfab.com/3d-models/tomato-a64e1ac7f8c44821bbc365f0581d95b2** (CC-BY, free, 1.3k tris, lightweight enough to scatter dozens across a scene).

**Tomato plant:** Either the paid **Vegetable Plants** Asset Store pack (**https://assetstore.unity.com/packages/3d/vegetation/plants/vegetable-plants-update-2394** — fastest path, includes matching tomato-plant + tomato-fruit prefabs with LODs) **or** a procedural stem+leaves+fruit build using the free CC-BY tomato mesh above (fully free, more setup work, more control over fruit placement for your localization ground truth).

**Static obstacles:** Unity primitives (`Cube`, `Cylinder`) with analytic colliders — free, zero import overhead, cheapest possible collision checks.

**Dynamic obstacle:** Unity `Capsule` primitive on a `NavMeshAgent` or scripted waypoint path crossing the UR10e workspace; optionally re-skin with a free Mixamo-rigged human (**https://www.mixamo.com**) purely for visual polish, keeping the capsule collider underneath for all avoidance logic.

**Camera:** **Intel RealSense D435** via the official `realsense2_description` package (**https://github.com/IntelRealSense/realsense-ros**, Apache-2.0, confirmed ROS1 Noetic/Melodic/Kinetic support) for correct housing geometry and RGB/depth extrinsic offsets, driven by two Unity `Camera` components (one for RGB, one reading `Camera.depthTexture` at the depth optical frame) published to ROS1 via **ROS-TCP-Connector/Endpoint** (part of **https://github.com/Unity-Technologies/Unity-Robotics-Hub**); optionally add Unity's **Perception package** (**https://github.com/Unity-Technologies/com.unity.perception**) on the RGB camera for ground-truth bounding-box/segmentation labels to validate your tomato detector. This camera+ROS1 publishing pattern mirrors what Unity's own official **Robotics Object Pose Estimation** demo (**https://github.com/Unity-Technologies/Robotics-Object-Pose-Estimation**) already implements for a UR3 + Robotiq gripper; you are effectively re-skinning that proven template with a UR10e + OnRobot RG2, a D435, and tomatoes instead of a UR3 + Robotiq and a cube.

**Why this set is technically coherent:** every robotics component (UR10e, RG2/RG6) is a standard ROS1-compatible URDF package built on the same xacro/mesh conventions the Unity URDF Importer expects; the camera pipeline reuses Unity's own official, already-working ROS1 example rather than an untested third-party sensor package; and the environment assets (tomatoes, plants, obstacles) are chosen or built specifically to keep collision geometry simple and localization ground-truth controllable — directly matching your stated priority order (ROS1 compatibility → Unity compatibility → correct joints → collision detection → path planning → detection/localization → obstacle avoidance → harvesting → visual quality last).
