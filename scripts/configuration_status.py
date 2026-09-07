#!/usr/bin/env python3
"""Report the immutable identity resolved by this launch, with graph freshness."""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node("humanoid_configuration_status")
    identity = json.loads(node.declare_parameter("identity", "{}").value)
    expected = node.declare_parameter("expected_nodes", ["humanoid_driver_runtime", "humanoid_motion_control"]).value
    publisher = node.create_publisher(String, "/humanoid/configuration_state", 10)
    started = time.time()

    def publish():
        present = set(node.get_node_names())
        missing = [name for name in expected if name not in present]
        payload = {**identity, "started_at": started, "stamp": time.time(),
                   "expected_nodes": list(expected), "missing_nodes": missing,
                   "state": "observed" if not missing else "starting"}
        publisher.publish(String(data=json.dumps(payload)))

    node.create_timer(1.0, publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
