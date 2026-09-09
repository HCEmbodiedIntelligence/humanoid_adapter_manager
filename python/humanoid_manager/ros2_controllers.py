"""Idempotent ros2_control initialization for any driver plugin's startup steps."""


def ensure_controllers(api, node, manager, names, timeout=60.0):
    def states():
        return {item.name: item.state for item in api.list_controllers(
            node, manager, timeout).controller}

    current = states()
    activate = []
    for name in names:
        state = current.get(name)
        if state is None:
            if not api.load_controller(node, manager, name, timeout).ok:
                raise RuntimeError(f'Failed to load controller: {name}')
            state = 'unconfigured'
        if state == 'unconfigured':
            if not api.configure_controller(node, manager, name, timeout).ok:
                raise RuntimeError(f'Failed to configure controller: {name}')
            state = 'inactive'
        if state == 'inactive':
            activate.append(name)
        elif state != 'active':
            raise RuntimeError(f'Controller {name} has unsupported state: {state}')
    if activate and not api.switch_controllers(node, manager, [], activate, True, True, 5.0).ok:
        raise RuntimeError(f'Failed to activate controllers: {activate}')
    current = states()
    missing = [name for name in names if current.get(name) != 'active']
    if missing:
        raise RuntimeError(f'Controllers did not become active: {missing}')


def main():
    import argparse
    import math
    import sys
    import rclpy
    from rclpy.utilities import remove_ros_args
    from rclpy.node import Node

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('controllers', nargs='+')
    parser.add_argument('--controller-manager', default='/controller_manager')
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args(remove_ros_args()[1:])
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    if len(set(args.controllers)) != len(args.controllers):
        parser.error('controller names must be unique')
    try:
        import controller_manager as api
    except ImportError:
        print('This plugin startup step requires the ROS controller_manager package.', file=sys.stderr)
        return 1
    rclpy.init()
    node = Node('ensure_controllers')
    try:
        ensure_controllers(api, node, args.controller_manager, args.controllers, args.timeout)
        node.get_logger().info('Controllers active: ' + ', '.join(args.controllers))
        return 0
    except Exception as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
