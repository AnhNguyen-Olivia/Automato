#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import math
from moveit_msgs.msg import RobotState, PlanningScene, PlanningSceneComponents, AllowedCollisionEntry
from moveit_msgs.srv import (GetPlanningScene, GetPlanningSceneRequest, ApplyPlanningScene,
                             GetStateValidity, GetStateValidityRequest)
from ur10e_rg2_moveit.srv import HarvestPose, HarvestPoseResponse

# Unity must send a gripper_position below this value to request a state reset
# (e.g. -999). Normal finger angles (grip_open = -0.2284, grip_close = 0.7853)
# are all above it, so they no longer trigger a reset by accident.
RESET_SENTINEL = -100.0

# Max allowed joint change between two consecutive Cartesian points (rad).
# With 5 mm steps anything bigger than this is almost certainly an IK flip.
MAX_JOINT_JUMP = 0.2

BASE_LINKS = ["base_link", "base_link_inertia", "base", "shoulder_link"]


def pose_distance(p1, p2):
    return math.sqrt(
        (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2
    )


def max_joint_jump(traj):
    """Largest single-joint change between consecutive trajectory points."""
    pts = traj.joint_trajectory.points
    return max(
        (max(abs(a - b) for a, b in zip(p1.positions, p0.positions))
         for p0, p1 in zip(pts, pts[1:])),
        default=0.0,
    )


def set_allowed(acm, a, b, allowed=True):
    for n in (a, b):
        if n not in acm.entry_names:
            acm.entry_names.append(n)
            for row in acm.entry_values:
                row.enabled.append(False)
            acm.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
    i, j = acm.entry_names.index(a), acm.entry_names.index(b)
    acm.entry_values[i].enabled[j] = allowed
    acm.entry_values[j].enabled[i] = allowed


def allow_base_on_objects(object_ids):
    """Let the robot base touch the given scene objects (e.g. the table)."""
    if not object_ids:
        return
    rospy.wait_for_service("/get_planning_scene", timeout=5.0)
    rospy.wait_for_service("/apply_planning_scene", timeout=5.0)
    get = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
    apply_scene = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)

    req = GetPlanningSceneRequest()
    req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    acm = get(req).scene.allowed_collision_matrix

    for obj in object_ids:
        for link in BASE_LINKS:
            set_allowed(acm, obj, link, True)

    scene = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
    apply_scene(scene)


class HarvestPoseServer:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.arm_group = moveit_commander.MoveGroupCommander("arm_group")
        self.arm_group.set_planner_id("RRTConnect")
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(5)
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)
        self.scene = moveit_commander.PlanningSceneInterface()

        # Tracked state: Unity moves the arm, ROS never executes, so we remember
        # where the last planned trajectory ended.
        self.last_joint_names = None
        self.last_joint_positions = None
        self.last_pose = None

        self.service = rospy.Service(
            "harvest_pose", HarvestPose, self.handle_request
        )
        rospy.loginfo("HarvestPose service ready.")

    def make_tracked_start_state(self):
        """Current robot state, with the arm joints overridden by the tracked values."""
        state = self.arm_group.get_current_state()
        names = list(state.joint_state.name)
        positions = list(state.joint_state.position)
        for n, p in zip(self.last_joint_names, self.last_joint_positions):
            if n in names:
                positions[names.index(n)] = p
        state.joint_state.position = positions
        return state

    def diagnose_cartesian_failure(self, target_pose, start_state, fraction, plan):
        """Explain WHY a Cartesian path stopped early. Log lines are prefixed [Diag].
        Wrapped so it can never break the request."""
        try:
            rospy.logwarn("[Diag] Cartesian with collisions: fraction %.2f, %d points",
                          fraction, len(plan.joint_trajectory.points))

            # 1) Same path with collision checking OFF
            plan_nc, frac_nc = self.arm_group.compute_cartesian_path(
                [target_pose], 0.005, avoid_collisions=False)
            rospy.logwarn("[Diag] Cartesian WITHOUT collision check: fraction %.2f, %d points",
                          frac_nc, len(plan_nc.joint_trajectory.points))
            if frac_nc < 0.999:
                rospy.logwarn("[Diag] -> Not a collision: IK / reach / joint limit.")
                if plan.joint_trajectory.points:
                    rospy.logwarn("[Diag] Last reachable joints: %s",
                                  [round(q, 3) for q in plan.joint_trajectory.points[-1].positions])
                return
            rospy.logwarn("[Diag] -> Collision-related. Looking for the colliding bodies...")

            # 2) Table as MoveIt sees it
            for name, o in self.scene.get_objects(['Table']).items():
                if o.primitives and o.primitive_poses:
                    p = o.primitive_poses[0].position
                    d = o.primitives[0].dimensions
                    rospy.logwarn("[Diag] %s: centre=(%.3f, %.3f, %.3f) dims=%s top z=%.3f frame=%s",
                                  name, p.x, p.y, p.z, [round(x, 3) for x in d],
                                  p.z + d[2] / 2.0, o.header.frame_id)
            rospy.logwarn("[Diag] Target flange pos=(%.3f, %.3f, %.3f)",
                          target_pose.position.x, target_pose.position.y, target_pose.position.z)

            # 3) Ask MoveIt which bodies collide at the first invalid point
            rospy.wait_for_service("/check_state_validity", timeout=3.0)
            check = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
            jt = plan_nc.joint_trajectory
            first_bad = None
            for idx, pt in enumerate(jt.points):
                st = RobotState()
                st.joint_state = start_state.joint_state
                names = list(st.joint_state.name)
                positions = list(st.joint_state.position)
                for n, q in zip(jt.joint_names, pt.positions):
                    if n in names:
                        positions[names.index(n)] = q
                st.joint_state.position = positions
                r = GetStateValidityRequest()
                r.robot_state = st
                r.group_name = "arm_group"
                res = check(r)
                if not res.valid:
                    first_bad = idx
                    rospy.logwarn("[Diag] First invalid point: %d of %d", idx, len(jt.points))
                    for c in res.contacts:
                        rospy.logwarn("[Diag]   contact: %s <-> %s  depth=%.4f  at (%.3f, %.3f, %.3f)",
                                      c.contact_body_1, c.contact_body_2, c.depth,
                                      c.position.x, c.position.y, c.position.z)
                    break
            if first_bad is None:
                rospy.logwarn("[Diag] Every point of the no-check path is valid; "
                              "the planner's own check disagrees (padding?).")
        except Exception as e:
            rospy.logwarn("[Diag] diagnostics failed: %s", e)

    def handle_request(self, req):
        response = HarvestPoseResponse()
        try:
            return self._handle(req, response)
        except Exception as e:
            rospy.logerr("handle_request crashed: %s", e)
            response.success = False
            response.message = "Server error: {}".format(e)
            return response

    def _handle(self, req, response):
        if req.gripper_position < RESET_SENTINEL:   # Unity restarted: forget tracked state
            self.last_joint_names = None
            self.last_joint_positions = None
            self.last_pose = None
            response.success = True
            response.message = "Tracked state reset."
            rospy.loginfo(response.message)
            return response

        try:
            self.prepare_scene()
        except Exception as e:
            rospy.logwarn("Could not prepare scene: %s", e)

        rospy.loginfo(
            "Received request: pos=(%.3f, %.3f, %.3f) gripper=%.3f",
            req.target_pose.position.x,
            req.target_pose.position.y,
            req.target_pose.position.z,
            req.gripper_position,
        )

        if self.last_joint_positions is not None:
            start_state = self.make_tracked_start_state()
            self.arm_group.set_start_state(start_state)
            start_pos = self.last_pose.position
            rospy.loginfo("Planning from tracked state.")
        else:
            self.arm_group.set_start_state_to_current_state()
            start_state = self.arm_group.get_current_state()
            start_pos = self.arm_group.get_current_pose().pose.position
            rospy.loginfo("Planning from current (initial) state.")

        dist = pose_distance(start_pos, req.target_pose.position)
        rospy.loginfo("Move distance: %.3fm", dist)

        arm_plan = None
        plan_success = False
        error_code = None

        # Always try a straight line first: it keeps the tool orientation and is dense (5 mm steps).
        short_move = dist < 0.15
        cartesian_plan, fraction = self.arm_group.compute_cartesian_path(
            [req.target_pose], 0.005, avoid_collisions=True)   # Noetic: no jump_threshold argument
        rospy.loginfo("Cartesian path fraction: %.2f", fraction)

        # Reject paths where the IK flipped between two neighbouring points.
        if fraction > 0:
            jump = max_joint_jump(cartesian_plan)
            rospy.loginfo("Max joint jump: %.3f rad", jump)
            if jump > MAX_JOINT_JUMP:
                rospy.logwarn("IK flip detected, rejecting Cartesian path.")
                fraction = 0.0

        # Short moves must be straight. Long moves must be almost complete, otherwise fall back to RRTConnect.
        min_fraction = 0.9 if short_move else 0.99

        if short_move and fraction < min_fraction:
            response.success = False
            response.message = "Cartesian path incomplete ({:.0f}%).".format(fraction * 100)
            rospy.logwarn(response.message)
            self.diagnose_cartesian_failure(req.target_pose, start_state, fraction, cartesian_plan)
            return response

        if fraction >= min_fraction:
            scale = 0.3
            arm_plan = self.arm_group.retime_trajectory(
                start_state,
                cartesian_plan,
                velocity_scaling_factor=scale,
                acceleration_scaling_factor=scale,
            )
            plan_success = True
        else:
            rospy.logwarn("Straight line not possible (%.2f), falling back to RRTConnect.", fraction)
            for attempt in (1, 2):
                self.arm_group.set_pose_target(req.target_pose)
                rospy.loginfo("Planning arm move (attempt %d)...", attempt)
                plan_success, arm_plan, planning_time, error_code = self.arm_group.plan()
                self.arm_group.clear_pose_targets()
                rospy.loginfo("Plan result: success=%s, error_code=%s", plan_success, error_code)
                if plan_success:
                    break
                rospy.sleep(0.5)

        if not plan_success:
            response.success = False
            response.message = "Arm planning failed (error_code={}).".format(error_code)
            rospy.logwarn(response.message)
            return response

        response.trajectory = arm_plan.joint_trajectory
        n = len(arm_plan.joint_trajectory.points)
        rospy.loginfo("Trajectory has %d points", n)
        if n == 0:
            response.success = False
            response.message = "Planner returned an empty trajectory."
            rospy.logwarn(response.message)
            return response

        self.last_joint_names = list(arm_plan.joint_trajectory.joint_names)
        self.last_joint_positions = list(arm_plan.joint_trajectory.points[-1].positions)
        self.last_pose = req.target_pose

        # The gripper is driven in Unity (ApplyGripper); nothing to plan here.
        response.success = True
        response.message = "Arm plan OK (gripper handled in Unity)."
        rospy.loginfo("Request complete: %s", response.message)
        return response

    def prepare_scene(self, timeout=3.0):
        """Wait until Unity's obstacles are in MoveIt, then let the base touch them."""
        t0 = rospy.Time.now()
        ids = []
        while (rospy.Time.now() - t0).to_sec() < timeout:
            ids = self.scene.get_known_object_names()
            if ids:
                break
            rospy.sleep(0.1)
        rospy.loginfo("Scene objects: %s", ids)
        if ids:
            allow_base_on_objects(ids)
        return ids


if __name__ == "__main__":
    rospy.init_node("harvest_pose_server")
    HarvestPoseServer()
    rospy.spin()
