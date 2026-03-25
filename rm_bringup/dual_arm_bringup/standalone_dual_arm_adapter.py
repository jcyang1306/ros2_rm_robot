#!/usr/bin/env python3
"""
Standalone dual-arm bridge: FollowJointTrajectory on the aggregate controller,
dispatch to left/right arm actions, and publish merged /joint_states.
"""
from __future__ import annotations

import logging
import time
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import CancelResponse, GoalResponse, ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class StandaloneDualArmAdapter:
    """
    Handles action ``dual_arm_controller/follow_joint_trajectory``, splits trajectories
    to left and right ``left_arm_controller`` and ``right_arm_controller`` actions, and publishes combined
    ``JointState`` from ``left_arm_controller/joint_states`` and ``right_arm_controller/joint_states``.
    arm_prefix format: left_arm_controller, right_arm_controller, dual_arm_controller
    """

    def __init__(
        self,
        node: Node,
        executor,
        *,
        joint_names: Optional[List[str]] = None,
        follow_joint_trajectory_action: str = "dual_arm_controller/follow_joint_trajectory",
        joint_states_topic: str = "/joint_states",
        left_arm_action: str = "/left_arm_controller/follow_joint_trajectory",
        right_arm_action: str = "/right_arm_controller/follow_joint_trajectory",
        left_joint_states_topic: str = "left_arm_controller/joint_states",
        right_joint_states_topic: str = "right_arm_controller/joint_states",
        joint_state_publish_period_sec: float = 0.1,
        dof: int = 12,
        left_dof: int = 6,
    ):
        self._node = node
        self._executor = executor
        self._dof = dof
        self._left_dof = left_dof
        self._right_dof = dof - left_dof
        self._logger = logging.getLogger(self.__class__.__name__)

        self._joint_names: List[str] = joint_names or [
            "left_joint1",
            "left_joint2",
            "left_joint3",
            "left_joint4",
            "left_joint5",
            "left_joint6",
            "right_joint1",
            "right_joint2",
            "right_joint3",
            "right_joint4",
            "right_joint5",
            "right_joint6",
        ]
        if len(self._joint_names) != dof:
            raise ValueError(f"joint_names length {len(self._joint_names)} != dof {dof}")

        self._jnt_pos = [0.0] * dof

        self._left_client = ActionClient(
            node,
            FollowJointTrajectory,
            left_arm_action,
        )
        self._right_client = ActionClient(
            node,
            FollowJointTrajectory,
            right_arm_action,
        )

        self._cb_reentrant = ReentrantCallbackGroup()
        self._cb_mutex = MutuallyExclusiveCallbackGroup()

        self._action_server = ActionServer(
            node,
            FollowJointTrajectory,
            follow_joint_trajectory_action,
            execute_callback=self._execute_trajectory_goal,
            callback_group=self._cb_reentrant,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        self._js_pub = node.create_publisher(JointState, joint_states_topic, 20)
        self._js_timer = node.create_timer(
            joint_state_publish_period_sec,
            self._publish_joint_states,
            callback_group=self._cb_mutex,
        )

        node.create_subscription(
            JointState,
            left_joint_states_topic,
            self._on_left_joint_states,
            10,
            callback_group=self._cb_mutex,
        )
        node.create_subscription(
            JointState,
            right_joint_states_topic,
            self._on_right_joint_states,
            10,
            callback_group=self._cb_mutex,
        )

    @property
    def joint_names(self) -> List[str]:
        return list(self._joint_names)

    def _on_left_joint_states(self, msg: JointState) -> None:
        # Match dual_arm_adapter: assume arm publishes one position per joint
        self._jnt_pos[: self._left_dof] = msg.position

    def _on_right_joint_states(self, msg: JointState) -> None:
        self._jnt_pos[self._left_dof :] = msg.position

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = list(self._jnt_pos)
        self._js_pub.publish(msg)

    def _goal_cb(self, goal: FollowJointTrajectory.Goal) -> GoalResponse:
        traj = goal.trajectory
        self._node.get_logger().info(
            f"Goal: {len(traj.points)} points, joints={list(traj.joint_names)}"
        )
        if not traj.points:
            self._node.get_logger().warn("Rejecting empty trajectory")
            return GoalResponse.REJECT
        if len(traj.joint_names) != self._dof:
            self._node.get_logger().warn(
                f"Rejecting trajectory: expected {self._dof} joint names, "
                f"got {len(traj.joint_names)}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle: ServerGoalHandle) -> CancelResponse:
        self._node.get_logger().info("Cancel requested (not forwarded to arm controllers)")
        return CancelResponse.ACCEPT

    async def _execute_trajectory_goal(self, goal_handle: ServerGoalHandle):
        trajectory = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()
        success = await self._dispatch_split_trajectory(trajectory)
        if success and rclpy.ok():
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            self._node.get_logger().info("Dual-arm trajectory succeeded")
        else:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            self._node.get_logger().warn("Dual-arm trajectory failed or aborted")
        return result

    def _split_trajectory(self, trajectory: JointTrajectory) -> tuple[JointTrajectory, JointTrajectory]:
        left_traj = JointTrajectory()
        right_traj = JointTrajectory()
        left_traj.header = trajectory.header
        right_traj.header = trajectory.header
        left_traj.joint_names = list(trajectory.joint_names[: self._left_dof])
        right_traj.joint_names = list(trajectory.joint_names[self._left_dof :])

        for point in trajectory.points:
            lp = JointTrajectoryPoint()
            lp.positions = list(point.positions[: self._left_dof])
            lp.velocities = list(point.velocities[: self._left_dof]) if point.velocities else []
            lp.accelerations = (
                list(point.accelerations[: self._left_dof]) if point.accelerations else []
            )
            lp.time_from_start = point.time_from_start
            left_traj.points.append(lp)

            rp = JointTrajectoryPoint()
            rp.positions = list(point.positions[self._left_dof :])
            rp.velocities = (
                list(point.velocities[self._left_dof :]) if point.velocities else []
            )
            rp.accelerations = (
                list(point.accelerations[self._left_dof :]) if point.accelerations else []
            )
            rp.time_from_start = point.time_from_start
            right_traj.points.append(rp)

        return left_traj, right_traj

    async def _dispatch_split_trajectory(self, trajectory: JointTrajectory) -> bool:
        left_traj, right_traj = self._split_trajectory(trajectory)

        left_goal = FollowJointTrajectory.Goal()
        left_goal.trajectory = left_traj
        left_goal.goal_time_tolerance = Duration(sec=0, nanosec=0)

        right_goal = FollowJointTrajectory.Goal()
        right_goal.trajectory = right_traj
        right_goal.goal_time_tolerance = Duration(sec=0, nanosec=0)

        if not self._left_client.wait_for_server(timeout_sec=5.0):
            self._logger.error("Left arm action server not available")
            return False
        if not self._right_client.wait_for_server(timeout_sec=5.0):
            self._logger.error("Right arm action server not available")
            return False

        t0 = time.time()
        # Kick off both sends before awaiting either — the arm controllers then
        # execute in parallel. asyncio.gather cannot be used here because rclpy's
        # MultiThreadedExecutor runs callbacks in a plain ThreadPoolExecutor that
        # has no asyncio event loop; rclpy Futures are awaitable directly instead.
        left_send_fut = self._left_client.send_goal_async(left_goal)
        right_send_fut = self._right_client.send_goal_async(right_goal)
        left_handle = await left_send_fut
        right_handle = await right_send_fut

        if left_handle is None or right_handle is None:
            self._logger.error("Goal send timed out or failed")
            return False
        if not left_handle.accepted or not right_handle.accepted:
            self._logger.error("One or both goals were rejected")
            return False

        left_res_fut = left_handle.get_result_async()
        right_res_fut = right_handle.get_result_async()
        left_res = await left_res_fut
        right_res = await right_res_fut

        if left_res is None or right_res is None:
            self._logger.error("Result wait timed out")
            return False

        ok = (
            left_res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            and right_res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        elapsed = time.time() - t0
        self._node.get_logger().info(
            f"Trajectory dispatch completed in {elapsed:.3f}s, success={ok}"
        )
        return ok


def main(args=None):
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = None
    try:
        node = rclpy.create_node("standalone_dual_arm_adapter")
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        StandaloneDualArmAdapter(node, executor)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
