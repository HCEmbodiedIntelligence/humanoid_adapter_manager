#!/usr/bin/env python3
"""Publish synthetic ROS data and optionally open a ready-to-record demo dashboard."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time


PREFIX = '/recording_test'
TOPICS = {
    'joint_states': 'sensor_msgs/msg/JointState',
    'joint_commands': 'sensor_msgs/msg/JointState',
    'gripper_states': 'sensor_msgs/msg/JointState',
    'gripper_commands': 'sensor_msgs/msg/JointState',
    'buttons': 'std_msgs/msg/String',
}
JOINT_NAMES = [f'openarmx_{side}_joint{i}' for side in ('left', 'right') for i in range(1, 8)]
GRIPPER_NAMES = ['left_gripper', 'right_gripper']


def finite_number(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError('必须为非负有限数值')
    return number


def prepare_dashboard(state_root, domain_id, port):
    """Create an independent demo configuration; never overwrite existing settings."""
    package = Path(__file__).resolve().parents[1]
    launcher = package / 'start_configurator.sh'
    if not launcher.is_file():
        raise ValueError('带网页运行请使用源码目录中的 scripts/simulate_recording.py')
    if state_root is None:
        parent = Path.home() / '.local/share/humanoid-manager-recording-demos'
        parent.mkdir(parents=True, exist_ok=True)
        state_root = Path(tempfile.mkdtemp(prefix='demo-', dir=parent))
    state_root = state_root.expanduser().resolve()
    state_root.mkdir(parents=True, exist_ok=True)
    (state_root / 'plugins').mkdir(exist_ok=True)
    config = {
        'server': {'host': '127.0.0.1', 'port': port},
        'adapter_manager': {
            'enabled': True, 'cli': str(package / 'scripts/humanoid_pluginctl.py'),
            'plugin_root': str(state_root / 'plugins'),
            'state_root': str(state_root / 'configuration'),
        },
        'data_quality': {
            'button_topic': PREFIX + '/buttons', 'button_enabled': True,
            'controller': 'right', 'button': 'secondary', 'scope': 'segment',
        },
        'ros': {
            'enabled': True, 'domain_id': domain_id, 'node_name': 'recording_demo_observer',
            'recording': {'directory': str(state_root / 'recordings')},
            'subscriptions': [
                {'topic': PREFIX + '/' + name, 'type': kind, 'enabled': True,
                 'outputs': ['record', 'websocket'], 'max_hz': 0, 'event_max_hz': 20}
                for name, kind in TOPICS.items()
            ],
        },
    }
    # JSON is valid YAML, so preparing the demo needs only the Python standard library.
    with (state_root / 'configurator.yaml').open('x', encoding='utf-8') as stream:
        json.dump(config, stream, ensure_ascii=False, indent=2)
    print(f'录制目录：{state_root / "recordings"}', flush=True)
    print(f'测试网页：http://127.0.0.1:{port}/dashboard/#topics', flush=True)
    return subprocess.Popen([str(launcher), '--state-root', str(state_root),
                             '--domain-id', str(domain_id), '--port', str(port)])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--web', action='store_true', help='同时启动预选好测试话题的独立网页')
    parser.add_argument('--domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    parser.add_argument('--port', type=int, default=7877, help='独立测试网页端口，默认 7877')
    parser.add_argument('--state-root', type=Path, help='新的网页状态目录；默认创建独立持久目录')
    parser.add_argument('--duration', type=finite_number, default=0, help='运行秒数，0 为持续运行')
    parser.add_argument('--hz', type=finite_number, default=100, help='关节发布频率，默认 100 Hz')
    parser.add_argument('--pause-after', type=finite_number, default=0, help='运行多少秒后暂停一次发布')
    parser.add_argument('--pause-for', type=finite_number, default=0, help='暂停秒数，用于测试话题中断')
    args = parser.parse_args(argv)
    if not 0 <= args.domain_id <= 232 or not 1 <= args.port <= 65535 or not 1 <= args.hz <= 1000:
        parser.error('domain-id 应为 0–232，port 为 1–65535，hz 为 1–1000')
    if bool(args.pause_after) != bool(args.pause_for):
        parser.error('暂停测试需要同时设置正数 --pause-after 和 --pause-for')
    if args.state_root and not args.web:
        parser.error('--state-root 需要配合 --web')

    # Keep the publisher and inherited web/recording processes in the same domain.
    # Passing domain_id to rclpy alone does not configure child environments.
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    class Source(Node):
        def __init__(self):
            super().__init__('recording_test_source')
            self.started = time.monotonic()
            self.sequence = 0
            self.last_held = False
            self.counts = dict.fromkeys(TOPICS, 0)
            self.publishers_by_name = {
                name: self.create_publisher(String if name == 'buttons' else JointState,
                                            PREFIX + '/' + name, 100)
                for name in TOPICS
            }
            self.create_timer(1 / args.hz, self.joints)
            self.create_timer(.05, self.grippers)
            self.create_timer(.05, self.buttons)

        def sample_time(self):
            elapsed = time.monotonic() - self.started
            if args.pause_for and args.pause_after <= elapsed < args.pause_after + args.pause_for:
                return None
            return elapsed, self.get_clock().now().to_msg()

        def publish(self, name, message):
            self.publishers_by_name[name].publish(message)
            self.counts[name] += 1

        def joint_message(self, names, stamp, positions, velocities):
            message = JointState(name=names, position=positions, velocity=velocities,
                                 effort=[0.] * len(names))
            message.header.stamp = stamp
            message.header.frame_id = 'synthetic_recording_test'
            return message

        def joints(self):
            sample = self.sample_time()
            if sample is None:
                return
            elapsed, stamp = sample
            for topic, lag in [('joint_commands', 0.), ('joint_states', .1)]:
                phases = [2 * math.pi * .2 * (elapsed - lag) + i * .15 for i in range(14)]
                self.publish(topic, self.joint_message(
                    JOINT_NAMES, stamp, [.2 * math.sin(p) for p in phases],
                    [.2 * 2 * math.pi * .2 * math.cos(p) for p in phases]))

        def grippers(self):
            sample = self.sample_time()
            if sample is None:
                return
            elapsed, stamp = sample
            for topic, lag in [('gripper_commands', 0.), ('gripper_states', .15)]:
                phases = [2 * math.pi * .1 * (elapsed - lag) + offset for offset in (0., math.pi)]
                self.publish(topic, self.joint_message(
                    GRIPPER_NAMES, stamp, [.022 * (1 - math.cos(p)) for p in phases],
                    [.022 * 2 * math.pi * .1 * math.sin(p) for p in phases]))

        def buttons(self):
            sample = self.sample_time()
            if sample is None:
                return
            elapsed, stamp = sample
            held = elapsed >= 5 and elapsed % 5 < .5
            pressed, released = held and not self.last_held, self.last_held and not held
            self.last_held = held
            self.sequence += 1
            def controller(active=False):
                return {'held_mask': 2 if active and held else 0,
                        'pressed_mask': 2 if active and pressed else 0,
                        'released_mask': 2 if active and released else 0,
                        'held': ['secondary'] if active and held else [],
                        'pressed': ['secondary'] if active and pressed else [],
                        'released': ['secondary'] if active and released else [],
                        'trigger': 0., 'grip': 0., 'primary_axis': [0., 0.], 'secondary_axis': [0., 0.]}
            edges = [{'controller': 'right', 'button': 'secondary', 'action': action}
                     for action, active in [('pressed', pressed), ('released', released)] if active]
            payload = {'synthetic': True, 'stamp_ns': stamp.sec * 10**9 + stamp.nanosec,
                       'sequence': self.sequence, 'vr_timestamp': elapsed,
                       'inputs': {'left': controller(), 'right': controller(True)}, 'edges': edges}
            self.publish('buttons', String(data=json.dumps(payload)))

    child, node = None, None
    try:
        if args.web:
            child = prepare_dashboard(args.state_root, args.domain_id, args.port)
        rclpy.init(args=[], domain_id=args.domain_id)
        node = Source()
        print(f'模拟发布已启动，ROS Domain ID={args.domain_id}；Ctrl+C 退出。', flush=True)
        for name in TOPICS:
            print(f'  {PREFIX}/{name}: {args.hz if name.startswith("joint_") else 20:g} Hz', flush=True)
        print('右手 B 键每 5 秒按下一次；测试网页已启用片段标记。' if args.web else
              '右手 B 键每 5 秒按下一次；可在录制设置中选择该按钮话题。', flush=True)
        while rclpy.ok() and (not args.duration or time.monotonic() - node.started < args.duration):
            if child is not None and child.poll() is not None:
                raise RuntimeError(f'测试网页已退出，退出码 {child.returncode}')
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            print('发布计数：' + json.dumps(node.counts), flush=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=20)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == '__main__':
    main()
