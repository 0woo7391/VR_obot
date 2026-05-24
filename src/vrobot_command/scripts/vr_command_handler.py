#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Sequence

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, MoveItErrorCodes, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass(frozen=True)
class ValidationResult:
    accepted: bool
    reason: str


class VrCommandHandler(Node):
    def __init__(self):
        super().__init__("vr_command_handler")
        self.declare_parameter("expected_frame_id", "base_link")
        self.declare_parameter("workspace_enabled", False)
        self.declare_parameter("workspace_min", [-1.0, -1.0, 0.0])
        self.declare_parameter("workspace_max", [1.0, 1.0, 1.5])
        self.declare_parameter("planning_group", "doosan_arm")
        self.declare_parameter("end_effector_link", "link_6")
        self.declare_parameter("planning_service_name", "/plan_kinematic_path")
        self.declare_parameter("execute_action_name", "/execute_trajectory")
        self.declare_parameter("execute_enabled", False)
        self.declare_parameter(
            "gripper_action_name",
            "/gripper_controller/follow_joint_trajectory",
        )
        self.declare_parameter("gripper_min_position", 0.0)
        self.declare_parameter("gripper_max_position", 1.1)
        self.declare_parameter("gripper_motion_duration", 1.0)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("planning_response_timeout", 6.0)
        self.declare_parameter("execution_response_timeout", 15.0)
        self.declare_parameter("communication_watchdog_enabled", False)
        self.declare_parameter("communication_watchdog_timeout", 0.5)
        self.declare_parameter("communication_watchdog_period", 0.1)
        self.declare_parameter("position_tolerance", 0.005)
        self.declare_parameter("orientation_tolerance", 0.01)

        self.expected_frame_id = self.get_parameter("expected_frame_id").value
        self.workspace_enabled = self.get_parameter("workspace_enabled").value
        self.workspace_min = self.get_parameter("workspace_min").value
        self.workspace_max = self.get_parameter("workspace_max").value
        self.planning_group = self.get_parameter("planning_group").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.planning_service_name = self.get_parameter("planning_service_name").value
        self.execute_action_name = self.get_parameter("execute_action_name").value
        self.execute_enabled = self.get_parameter("execute_enabled").value
        self.gripper_action_name = self.get_parameter("gripper_action_name").value
        self.gripper_min_position = self.get_parameter("gripper_min_position").value
        self.gripper_max_position = self.get_parameter("gripper_max_position").value
        self.gripper_motion_duration = self.get_parameter("gripper_motion_duration").value
        self.allowed_planning_time = self.get_parameter("allowed_planning_time").value
        self.planning_response_timeout = self.get_parameter("planning_response_timeout").value
        self.execution_response_timeout = self.get_parameter("execution_response_timeout").value
        self.communication_watchdog_enabled = self.get_parameter(
            "communication_watchdog_enabled"
        ).value
        self.communication_watchdog_timeout = self.get_parameter(
            "communication_watchdog_timeout"
        ).value
        self.communication_watchdog_period = self.get_parameter(
            "communication_watchdog_period"
        ).value
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.orientation_tolerance = self.get_parameter("orientation_tolerance").value

        self._validate_workspace_config()

        self.status_publisher = self.create_publisher(String, "/vr/command_status", 10)
        self.pose_goal_subscription = self.create_subscription(
            PoseStamped,
            "/vr/pose_goal",
            self._handle_pose_goal,
            10,
        )
        self.gripper_goal_subscription = self.create_subscription(
            Float64,
            "/vr/gripper_goal",
            self._handle_gripper_goal,
            10,
        )
        self.planning_client = self.create_client(GetMotionPlan, self.planning_service_name)
        self.execute_client = ActionClient(self, ExecuteTrajectory, self.execute_action_name)
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.gripper_action_name,
        )
        self.planning_timeout_timer = None
        self.execution_timeout_timer = None
        self.planning_timed_out = False
        self.execution_timed_out = False
        self.execution_goal_handle = None
        self.command_in_progress = False
        self.last_pose_goal_time = None
        self.communication_timed_out = False
        self.communication_watchdog_timer = self.create_timer(
            self.communication_watchdog_period,
            self._handle_communication_watchdog,
        )

        self.get_logger().info(
            "Ready for VR pose goals. "
            f"workspace_enabled={self.workspace_enabled}, "
            f"execute_enabled={self.execute_enabled}"
        )

    def _validate_workspace_config(self):
        if len(self.workspace_min) != 3 or len(self.workspace_max) != 3:
            raise ValueError("workspace_min and workspace_max must each contain 3 values")

        for minimum, maximum in zip(self.workspace_min, self.workspace_max):
            if minimum >= maximum:
                raise ValueError("Each workspace_min value must be lower than workspace_max")

        if self.gripper_min_position >= self.gripper_max_position:
            raise ValueError("gripper_min_position must be lower than gripper_max_position")

        if self.gripper_motion_duration <= 0.0:
            raise ValueError("gripper_motion_duration must be positive")

    def _handle_pose_goal(self, msg: PoseStamped):
        self.last_pose_goal_time = self.get_clock().now()
        self.communication_timed_out = False

        if self.command_in_progress:
            self._publish_status("REJECTED:COMMAND_IN_PROGRESS")
            return

        result = self._validate_pose_goal(msg)
        if not result.accepted:
            self._publish_status(f"REJECTED:{result.reason}")
            return

        self._publish_status("ACCEPTED")
        self._request_plan(msg)

    def _handle_gripper_goal(self, msg: Float64):
        if not self.gripper_min_position <= msg.data <= self.gripper_max_position:
            self._publish_status("REJECTED:GRIPPER_OUT_OF_RANGE")
            return

        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self._publish_status("FAILED:GRIPPER_SERVER_UNAVAILABLE")
            return

        self._publish_status("GRIPPER_ACCEPTED")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_gripper_trajectory(msg.data)
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self._handle_gripper_goal_response)

    def _build_gripper_trajectory(self, position: float) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = ["rh_r1", "rh_r2", "rh_l1", "rh_l2"]

        point = JointTrajectoryPoint()
        point.positions = [position, position, position, position]
        point.time_from_start = Duration(
            sec=int(self.gripper_motion_duration),
            nanosec=int((self.gripper_motion_duration % 1) * 1e9),
        )
        trajectory.points = [point]
        return trajectory

    def _handle_gripper_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status("FAILED:GRIPPER_REJECTED")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_gripper_result)

    def _handle_gripper_result(self, future):
        result = future.result()
        if result is None:
            self._publish_status("FAILED:GRIPPER_CALL_FAILED")
            return

        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self._publish_status("GRIPPER_EXECUTED")
            return

        self._publish_status(f"FAILED:GRIPPER_{result.result.error_code}")

    def _validate_pose_goal(self, msg: PoseStamped) -> ValidationResult:
        if msg.header.frame_id != self.expected_frame_id:
            return ValidationResult(False, "INVALID_FRAME")

        if self.workspace_enabled and not self._inside_workspace(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        ):
            return ValidationResult(False, "OUTSIDE_WORKSPACE")

        return ValidationResult(True, "VALID")

    def _inside_workspace(self, position: Sequence[float]) -> bool:
        return all(
            minimum <= value <= maximum
            for value, minimum, maximum in zip(
                position,
                self.workspace_min,
                self.workspace_max,
            )
        )

    def _request_plan(self, msg: PoseStamped):
        if not self.planning_client.wait_for_service(timeout_sec=1.0):
            self._publish_status("FAILED:PLANNING_SERVICE_UNAVAILABLE")
            return

        self.command_in_progress = True

        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = self.planning_group
        request.motion_plan_request.allowed_planning_time = self.allowed_planning_time
        request.motion_plan_request.num_planning_attempts = 1
        request.motion_plan_request.start_state.is_diff = True
        request.motion_plan_request.goal_constraints = [self._build_goal_constraints(msg)]

        future = self.planning_client.call_async(request)
        self._start_planning_timeout()
        future.add_done_callback(self._handle_plan_result)

    def _build_goal_constraints(self, msg: PoseStamped) -> Constraints:
        position_region = SolidPrimitive()
        position_region.type = SolidPrimitive.SPHERE
        position_region.dimensions = [self.position_tolerance]

        position_constraint = PositionConstraint()
        position_constraint.header = msg.header
        position_constraint.link_name = self.end_effector_link
        position_constraint.constraint_region.primitives = [position_region]
        position_constraint.constraint_region.primitive_poses = [msg.pose]
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = msg.header
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = msg.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
        return constraints

    def _handle_plan_result(self, future):
        if self.planning_timed_out:
            return
        self._cancel_timer("planning")
        response = future.result()
        if response is None:
            self.command_in_progress = False
            self._publish_status("FAILED:PLANNING_CALL_FAILED")
            return

        if response.motion_plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
            self._publish_status("PLANNED")
            if self.execute_enabled:
                self._request_execute(response.motion_plan_response.trajectory)
            else:
                self.command_in_progress = False
            return

        self.command_in_progress = False
        self._publish_status(
            f"FAILED:{self._moveit_error_name(response.motion_plan_response.error_code.val, 'PLANNING')}"
        )

    def _request_execute(self, trajectory):
        if not self.execute_client.wait_for_server(timeout_sec=1.0):
            self.command_in_progress = False
            self._publish_status("FAILED:EXECUTION_SERVER_UNAVAILABLE")
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self.execute_client.send_goal_async(goal)
        self._start_execution_timeout()
        future.add_done_callback(self._handle_execute_goal_response)

    def _handle_execute_goal_response(self, future):
        if self.execution_timed_out:
            return
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._cancel_timer("execution")
            self.command_in_progress = False
            self._publish_status("FAILED:EXECUTION_REJECTED")
            return

        self.execution_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_execute_result)

    def _handle_execute_result(self, future):
        if self.execution_timed_out:
            return
        self._cancel_timer("execution")
        self.execution_goal_handle = None
        self.command_in_progress = False
        result = future.result()
        if result is None:
            self._publish_status("FAILED:EXECUTION_CALL_FAILED")
            return

        if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            self._publish_status("EXECUTED")
            return

        self._publish_status(
            f"FAILED:{self._moveit_error_name(result.result.error_code.val, 'EXECUTION')}"
        )

    def _start_planning_timeout(self):
        self._cancel_timer("planning")
        self.planning_timed_out = False
        self.planning_timeout_timer = self.create_timer(
            self.planning_response_timeout,
            self._handle_planning_timeout,
        )

    def _start_execution_timeout(self):
        self._cancel_timer("execution")
        self.execution_timed_out = False
        self.execution_timeout_timer = self.create_timer(
            self.execution_response_timeout,
            self._handle_execution_timeout,
        )

    def _handle_planning_timeout(self):
        self.planning_timed_out = True
        self.command_in_progress = False
        self._cancel_timer("planning")
        self._publish_status("FAILED:PLANNING_TIMED_OUT")

    def _handle_execution_timeout(self):
        self.execution_timed_out = True
        self.command_in_progress = False
        self._cancel_timer("execution")
        self._publish_status("FAILED:EXECUTION_TIMED_OUT")

    def _handle_communication_watchdog(self):
        if (
            not self.communication_watchdog_enabled
            or not self.execute_enabled
            or self.execution_goal_handle is None
        ):
            return

        if self.last_pose_goal_time is None:
            return

        elapsed = (self.get_clock().now() - self.last_pose_goal_time).nanoseconds / 1e9
        if elapsed <= self.communication_watchdog_timeout or self.communication_timed_out:
            return

        self.communication_timed_out = True
        self.execution_timed_out = True
        self._cancel_timer("execution")
        self.execution_goal_handle.cancel_goal_async()
        self.execution_goal_handle = None
        self.command_in_progress = False
        self._publish_status("FAILED:COMMUNICATION_TIMED_OUT")

    def _cancel_timer(self, kind: str):
        timer = self.planning_timeout_timer if kind == "planning" else self.execution_timeout_timer
        if timer is None:
            return
        timer.cancel()
        if kind == "planning":
            self.planning_timeout_timer = None
        else:
            self.execution_timeout_timer = None

    def _moveit_error_name(self, value: int, phase: str) -> str:
        names = {
            MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
            MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "PLAN_INVALIDATED",
            MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
            MoveItErrorCodes.TIMED_OUT: f"{phase}_TIMED_OUT",
            MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
            MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
            MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
            MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
            MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
            MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
        }
        return names.get(value, f"{phase}_{value}")

    def _publish_status(self, value: str):
        status = String()
        status.data = value
        self.status_publisher.publish(status)
        self.get_logger().info(value)


def main():
    rclpy.init()
    node = VrCommandHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
