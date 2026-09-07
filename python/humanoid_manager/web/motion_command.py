"""Explicit web-initiated motion commands, isolated from configuration editing."""
from __future__ import annotations

import time
import uuid


class MotionCommandError(RuntimeError):
    pass


def _spin_until(executor, futures, deadline):
    while not all(future.done() for future in futures):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        executor.spin_once(timeout_sec=min(0.05, remaining))
    return True


def execute_move_j_pose(goals, domain_id):
    """Send all disjoint MoveJ targets together and wait for real action results."""
    if not goals:
        raise MotionCommandError("初始姿态没有可执行目标")
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from humanoid_motion_interfaces.action import MoveJ
        from humanoid_motion_interfaces.msg import Status
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
    except ImportError as error:
        raise MotionCommandError("当前环境未安装 humanoid_motion_interfaces，无法执行初始姿态") from error

    context, node, executor = Context(), None, None
    clients, handles = [], []
    try:
        rclpy.init(args=[], context=context, domain_id=int(domain_id))
        node = rclpy.create_node(f"humanoid_manager_pose_{uuid.uuid4().hex[:10]}", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        prepared = []
        for target in goals:
            client = ActionClient(node, MoveJ, target["endpoint"])
            clients.append(client)
            if not client.wait_for_server(timeout_sec=2.0):
                raise MotionCommandError(f"MoveJ 接口不可用: {target['endpoint']}")
            goal = MoveJ.Goal()
            goal.group_name = target["group"]
            goal.target.name = list(target["joint_names"])
            goal.target.position = [float(value) for value in target["positions_rad"]]
            goal.options.velocity_scale = float(target["velocity_scale"])
            goal.options.acceleration_scale = float(target["acceleration_scale"])
            goal.options.jerk_scale = float(target["jerk_scale"])
            goal.options.timeout_sec = float(target["timeout_sec"])
            prepared.append((target, client, goal))

        acceptance = [client.send_goal_async(goal) for _, client, goal in prepared]
        if not _spin_until(executor, acceptance, time.monotonic() + 5.0):
            raise MotionCommandError("等待 MoveJ 接收目标超时")
        for (target, _, _), future in zip(prepared, acceptance):
            error = future.exception()
            if error is not None:
                raise MotionCommandError(f"{target['endpoint']} 发送失败: {error}")
            handle = future.result()
            if not handle.accepted:
                raise MotionCommandError(f"MoveJ 拒绝初始姿态目标: {target['endpoint']}")
            handles.append((target, handle))

        results = [handle.get_result_async() for _, handle in handles]
        deadline = time.monotonic() + max(float(goal["timeout_sec"]) for goal in goals) + 10.0
        if not _spin_until(executor, results, deadline):
            cancellations = [handle.cancel_goal_async() for _, handle in handles]
            _spin_until(executor, cancellations, time.monotonic() + 2.0)
            raise MotionCommandError("执行初始姿态超时，已请求取消运动")

        output = []
        for (target, _), future in zip(handles, results):
            error = future.exception()
            if error is not None:
                raise MotionCommandError(f"{target['endpoint']} 执行失败: {error}")
            wrapped = future.result()
            status = wrapped.result.status
            item = {
                "channel": target["channel"],
                "endpoint": target["endpoint"],
                "group": target["group"],
                "goal_status": int(wrapped.status),
                "status_code": int(status.code),
                "message": status.message,
                "final_joint_state": {
                    "name": list(wrapped.result.final_joint_state.name),
                    "position": list(wrapped.result.final_joint_state.position),
                },
            }
            output.append(item)
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or status.code != Status.OK:
                raise MotionCommandError(
                    f"{target['group']} 未到达初始姿态: {status.message or 'MoveJ 返回失败'}"
                )
        return output
    except Exception as error:
        cancellations = []
        for _, handle in handles:
            try:
                cancellations.append(handle.cancel_goal_async())
            except Exception:
                pass
        if cancellations and executor is not None:
            _spin_until(executor, cancellations, time.monotonic() + 2.0)
        if isinstance(error, MotionCommandError):
            raise
        raise MotionCommandError(f"执行初始姿态时 ROS 2 通信失败: {error}") from error
    finally:
        if executor is not None and node is not None:
            executor.remove_node(node)
        for client in clients:
            try:
                client.destroy()
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
