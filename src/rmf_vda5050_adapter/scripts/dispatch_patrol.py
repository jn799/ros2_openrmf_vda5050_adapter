#!/usr/bin/env python3
"""
Send a patrol task to the RMF fleet adapter.

Usage:
  ros2 run rmf_vda5050_adapter dispatch_patrol <place1> [place2 ...]  [--rounds N]

Examples:
  ros2 run rmf_vda5050_adapter dispatch_patrol wp1
  ros2 run rmf_vda5050_adapter dispatch_patrol wp1 wp2 wp3 --rounds 2
  ros2 run rmf_vda5050_adapter dispatch_patrol charging

The place names must match waypoint names in nav_graph.yaml:
  start, wp1, wp2, wp3, charging
"""

import argparse
import json
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rmf_task_msgs.msg import ApiRequest


def main():
    parser = argparse.ArgumentParser(
        description='Dispatch a patrol task to the RMF VDA5050 adapter')
    parser.add_argument('places', nargs='+',
                        help='Waypoint name(s) to patrol, in order')
    parser.add_argument('--rounds', type=int, default=1,
                        help='How many times to complete the patrol (default: 1)')

    # Strip ROS args before parsing
    args = parser.parse_args(
        [a for a in sys.argv[1:] if not a.startswith('__')])

    rclpy.init()
    node = Node('dispatch_patrol')

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub = node.create_publisher(ApiRequest, 'task_api_requests', qos)

    request_id = f'patrol_{uuid.uuid4().hex[:8]}'
    payload = {
        'type': 'dispatch_task_request',
        'request': {
            'category': 'patrol',
            'description': {
                'places': args.places,
                'rounds': args.rounds,
            },
            'unix_millis_earliest_start_time': 0,
            'priority': {'type': 'default', 'value': 0},
            'labels': [],
        },
    }

    msg = ApiRequest()
    msg.request_id = request_id
    msg.json_msg = json.dumps(payload)

    # Give the publisher a moment to connect before sending
    time.sleep(0.5)
    pub.publish(msg)

    node.get_logger().info(
        f'Dispatched patrol to {args.places} '
        f'(rounds={args.rounds}, id={request_id})')

    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
