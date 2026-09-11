#!/usr/bin/env python3
"""Replace platform arm/gripper execution with delayed command feedback."""
import argparse
import math
import os
import time

from humanoid_manager.mock_feedback import ARM_NAMES, GRIPPER_NAMES, DelayedFeedback


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '199')))
    parser.add_argument('--arm-hz', type=float, default=100.)
    parser.add_argument('--gripper-hz', type=float, default=50.)
    parser.add_argument('--arm-joints', default=','.join(ARM_NAMES), help='Comma-separated platform joint names')
    parser.add_argument('--gripper-joints', default=','.join(GRIPPER_NAMES), help='Comma-separated names; empty disables grippers')
    parser.add_argument('--duration', type=float, default=0., help='Seconds; 0 runs until stopped')
    args, ros_args = parser.parse_known_args(argv)
    if ros_args and ros_args[0] != '--ros-args':
        parser.error(f'unrecognized arguments: {" ".join(ros_args)}')
    if not 0 <= args.domain_id <= 232:
        parser.error('--domain-id must be in 0–232')
    if any(not math.isfinite(hz) or not 0 < hz <= 1000 for hz in (args.arm_hz, args.gripper_hz)):
        parser.error('publish rates must be finite and in (0, 1000]')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    arm_names = args.arm_joints.split(',')
    gripper_names = args.gripper_joints.split(',') if args.gripper_joints else []
    if not all(arm_names) or len(set(arm_names + gripper_names)) != len(arm_names + gripper_names) or not all(gripper_names):
        parser.error('joint names must be nonempty and unique')

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    class FeedbackNode(Node):
        def __init__(self):
            super().__init__('mock_robot_feedback')
            self.counts = {'arm_commands': 0, 'gripper_commands': 0, 'rejected': 0}
            self.models = {
                'arm': DelayedFeedback(arm_names, [0.] * len(arm_names)),
                'gripper': DelayedFeedback(gripper_names, [.04] * len(gripper_names)),
            }
            self.outputs = {}
            for key, command, state, hz in (
                ('arm', '/hc_teleop/joint_cmd', '/hc_teleop/joint_states', args.arm_hz),
                ('gripper', '/hc_teleop/gripper_commands', '/hc_teleop/gripper_states', args.gripper_hz),
            ):
                if not self.models[key].names:
                    continue
                self.outputs[key] = self.create_publisher(JointState, state, 10)
                self.create_subscription(JointState, command,
                                         lambda msg, key=key: self.accept(key, msg), 10)
                self.create_timer(1 / hz, lambda key=key: self.publish_state(key))
            self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
            self.create_timer(.1, self.publish_diagnostics)

        def accept(self, key, message):
            try:
                self.models[key].accept(message.name, message.position, message.velocity, message.effort)
            except ValueError as error:
                self.counts['rejected'] += 1
                self.get_logger().warning(f'Rejected {key} command: {error}')
                return
            self.counts[key + '_commands'] += 1

        def publish_state(self, key):
            model = self.models[key]
            msg = JointState(name=list(model.names), position=model.tick(),
                             velocity=[0.] * len(model.names), effort=[0.] * len(model.names))
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'mock_robot_feedback'
            self.outputs[key].publish(msg)

        def publish_diagnostics(self):
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.status = [DiagnosticStatus(
                level=DiagnosticStatus.OK, name='mock_robot_feedback', hardware_id='simulation',
                message='Command feedback simulation; no physical execution',
                values=[KeyValue(key=k, value=str(v)) for k, v in self.counts.items()])]
            self.diagnostics.publish(msg)

    rclpy.init(args=ros_args, domain_id=args.domain_id)
    node = None
    try:
        node = FeedbackNode()
        print(f'Mock feedback ready: domain={args.domain_id}, arms={args.arm_hz:g} Hz, grippers={args.gripper_hz:g} Hz', flush=True)
        print('Command topics are inputs only. No driver, CAN, camera or VR process is started.', flush=True)
        started = time.monotonic()
        while rclpy.ok() and (not args.duration or time.monotonic() - started < args.duration):
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
