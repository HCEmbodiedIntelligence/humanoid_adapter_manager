"""Explicit web-initiated gripper checks through the managed platform topics."""
from __future__ import annotations

import math
import time
import uuid


class GripperCommandError(RuntimeError):
    pass


def execute_gripper_test(command, domain_id):
    """Publish a bounded target while observing fresh measured gripper feedback."""
    required = {"command_topic", "state_topic", "name", "position", "max_effort", "timeout_sec"}
    if not isinstance(command, dict) or set(command) != required:
        raise GripperCommandError("夹爪测试命令不完整")
    if not all(isinstance(command[key], str) and command[key] for key in
               ("command_topic", "state_topic", "name")):
        raise GripperCommandError("夹爪测试话题和名称不能为空")
    try:
        position = float(command["position"])
        effort = float(command["max_effort"])
        timeout = float(command["timeout_sec"])
    except (TypeError, ValueError) as error:
        raise GripperCommandError("夹爪测试目标必须是有限数值") from error
    if not all(math.isfinite(value) for value in (position, effort, timeout)):
        raise GripperCommandError("夹爪测试目标必须是有限数值")
    if effort < 0.0 or not 0.1 <= timeout <= 15.0:
        raise GripperCommandError("夹爪测试力度或超时无效")

    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
    except ImportError as error:
        raise GripperCommandError("当前环境缺少 ROS 2 JointState 支持，无法测试夹爪") from error

    context, node, executor = Context(), None, None
    publisher = subscription = None
    feedback = {"position": None, "count": 0}
    published = 0
    started = time.monotonic()
    try:
        rclpy.init(args=[], context=context, domain_id=int(domain_id))
        node = rclpy.create_node(
            f"humanoid_manager_gripper_test_{uuid.uuid4().hex[:10]}", context=context
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        def receive(message):
            if message.name.count(command["name"]) != 1:
                return
            index = message.name.index(command["name"])
            if index >= len(message.position) or not math.isfinite(message.position[index]):
                return
            feedback["position"] = float(message.position[index])
            feedback["count"] += 1

        publisher = node.create_publisher(JointState, command["command_topic"], 10)
        subscription = node.create_subscription(
            JointState, command["state_topic"], receive, qos_profile_sensor_data
        )
        ready_deadline = time.monotonic() + 3.0
        while (publisher.get_subscription_count() == 0 or feedback["position"] is None):
            remaining = ready_deadline - time.monotonic()
            if remaining <= 0:
                if publisher.get_subscription_count() == 0:
                    raise GripperCommandError("夹爪运行时未订阅测试命令话题")
                raise GripperCommandError("夹爪没有返回新鲜位置反馈，未发送测试命令")
            executor.spin_once(timeout_sec=min(0.05, remaining))

        initial = feedback["position"]
        tolerance = 0.001
        deadline = time.monotonic() + timeout
        while abs(feedback["position"] - position) > tolerance:
            if time.monotonic() >= deadline:
                raise GripperCommandError(
                    f"夹爪在 {timeout:g} 秒内未到达目标；已发送实测位置保持命令"
                )
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = [command["name"]]
            message.position = [position]
            message.effort = [effort]
            publisher.publish(message)
            published += 1
            executor.spin_once(timeout_sec=0.05)

        return {
            "name": command["name"],
            "initial_position": initial,
            "target_position": position,
            "final_position": feedback["position"],
            "feedback_messages": feedback["count"],
            "command_messages": published,
            "duration_seconds": round(time.monotonic() - started, 3),
        }
    except Exception as error:
        if isinstance(error, GripperCommandError):
            raise
        raise GripperCommandError(f"夹爪测试时 ROS 2 通信失败: {error}") from error
    finally:
        # If movement started, explicitly request a hold at the latest measured position.
        # The independent runtime watchdog remains the second line of protection.
        if published and publisher is not None and node is not None and feedback["position"] is not None:
            try:
                hold = JointState()
                hold.header.stamp = node.get_clock().now().to_msg()
                hold.name = [command["name"]]
                hold.position = [feedback["position"]]
                hold.effort = [effort]
                for _ in range(3):
                    publisher.publish(hold)
                    executor.spin_once(timeout_sec=0.02)
            except Exception:
                pass
        if executor is not None and node is not None:
            executor.remove_node(node)
        if node is not None:
            if subscription is not None:
                node.destroy_subscription(subscription)
            if publisher is not None:
                node.destroy_publisher(publisher)
            node.destroy_node()
        if context.ok():
            context.shutdown()
