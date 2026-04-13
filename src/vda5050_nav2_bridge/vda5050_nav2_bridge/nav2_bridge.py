"""
VDA5050 Nav2 Bridge

Bridges VDA5050 orders (received via MQTT from rmf_vda5050_adapter) to a
Nav2-enabled robot (e.g. TurtleBot3).

Behaviour:
  - Publishes VDA5050 'connection' ONLINE on startup.
  - Receives VDA5050 'order' messages and drives the robot via Nav2
    NavigateToPose action goals, one waypoint at a time.
  - Reports the robot's real pose (from /amcl_pose or /odom) in VDA5050 state.
  - Publishes VDA5050 'state' at a configurable rate.
  - Supports cancelOrder instant action.
"""

import json
import math
import threading
import time
import uuid

import paho.mqtt.client as mqtt
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


def _yaw_from_quaternion(q) -> float:
    """Extract yaw (radians) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw):
    """Return (qz, qw) for a pure yaw rotation."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Nav2Bridge(Node):

    def __init__(self):
        super().__init__('vda5050_nav2_bridge')

        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('manufacturer', 'turtlebot')
        self.declare_parameter('serial_number', 'tb3_01')
        self.declare_parameter('vda5050_version', 'v2')
        self.declare_parameter('nav2_action_server', 'navigate_to_pose')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('state_publish_rate', 1.0)
        self.declare_parameter('nav2_server_timeout', 10.0)
        self.declare_parameter('robot_series_name', 'TurtleBot3')
        self.declare_parameter('robot_length', 0.138)
        self.declare_parameter('robot_width', 0.178)
        self.declare_parameter('robot_height_max', 0.192)
        self.declare_parameter('robot_speed_max', 0.22)
        self.declare_parameter('robot_accel_max', 0.5)
        self.declare_parameter('robot_decel_max', 0.5)
        self.declare_parameter('robot_max_load_mass', 15.0)

        self._host = self.get_parameter('mqtt_host').value
        self._port = self.get_parameter('mqtt_port').value
        self._user = self.get_parameter('mqtt_username').value
        self._pass = self.get_parameter('mqtt_password').value
        self._manufacturer = self.get_parameter('manufacturer').value
        self._serial = self.get_parameter('serial_number').value
        self._version = self.get_parameter('vda5050_version').value
        self._action_server = self.get_parameter('nav2_action_server').value
        self._pose_topic = self.get_parameter('pose_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._state_rate = self.get_parameter('state_publish_rate').value
        self._nav2_timeout = self.get_parameter('nav2_server_timeout').value
        self._series_name    = self.get_parameter('robot_series_name').value
        self._robot_length   = self.get_parameter('robot_length').value
        self._robot_width    = self.get_parameter('robot_width').value
        self._robot_height   = self.get_parameter('robot_height_max').value
        self._speed_max      = self.get_parameter('robot_speed_max').value
        self._accel_max      = self.get_parameter('robot_accel_max').value
        self._decel_max      = self.get_parameter('robot_decel_max').value
        self._max_load_mass  = self.get_parameter('robot_max_load_mass').value

        # Robot state — protected by _lock
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._pose_initialized = False
        self._driving = False
        self._battery = 100.0
        self._order_id = ''
        self._last_node_id = ''
        self._errors: list = []
        self._header_counter = 0
        self._lock = threading.Lock()

        # Waypoint queue: list of {nodeId, x, y, theta}
        self._waypoint_queue: list[dict] = []
        self._motion_thread: threading.Thread | None = None

        # Current Nav2 goal handle (for cancellation)
        self._current_goal_handle = None
        self._cancel_requested = False

        # Nav2 action client
        self._nav2_client = ActionClient(self, NavigateToPose, self._action_server)

        # Pose subscription — /amcl_pose or /odom
        if 'amcl' in self._pose_topic:
            self.create_subscription(
                PoseWithCovarianceStamped,
                self._pose_topic,
                self._on_amcl_pose,
                10,
            )
        else:
            self.create_subscription(
                Odometry,
                self._pose_topic,
                self._on_odom,
                10,
            )

        self._setup_mqtt()

        interval = 1.0 / self._state_rate
        self.create_timer(interval, self._publish_state)

        self.get_logger().info(
            f"Nav2 bridge started: {self._manufacturer}/{self._serial} "
            f"| Nav2 server: '{self._action_server}' "
            f"| Pose topic: '{self._pose_topic}'"
        )

    # ------------------------------------------------------------------
    # Pose callbacks
    # ------------------------------------------------------------------

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        with self._lock:
            self._x = p.position.x
            self._y = p.position.y
            self._theta = _yaw_from_quaternion(p.orientation)
            self._pose_initialized = True

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        with self._lock:
            self._x = p.position.x
            self._y = p.position.y
            self._theta = _yaw_from_quaternion(p.orientation)
            self._pose_initialized = True

    # ------------------------------------------------------------------
    # MQTT
    # ------------------------------------------------------------------

    def _topic(self, kind: str) -> str:
        return f"uagv/{self._version}/{self._manufacturer}/{self._serial}/{kind}"

    def _setup_mqtt(self):
        self._mqtt = mqtt.Client(
            client_id=f"nav2bridge_{self._serial}_{uuid.uuid4().hex[:6]}"
        )
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_message = self._on_message
        self._mqtt.on_disconnect = self._on_disconnect

        if self._user:
            self._mqtt.username_pw_set(self._user, self._pass)

        try:
            self._mqtt.connect(self._host, self._port, keepalive=60)
        except Exception as e:
            self.get_logger().error(f"MQTT connection failed: {e}")
            return

        self._mqtt.loop_start()

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self.get_logger().error(f"MQTT connect refused, rc={rc}")
            return
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe(self._topic('order'), qos=1)
        client.subscribe(self._topic('instantActions'), qos=1)
        self._publish_connection('ONLINE')
        self._publish_factsheet()

    def _on_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT disconnected (rc={rc})")

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Bad JSON on {msg.topic}: {e}")
            return

        kind = msg.topic.split('/')[-1]
        if kind == 'order':
            self._handle_order(payload)
        elif kind == 'instantActions':
            self._handle_instant_actions(payload)

    # ------------------------------------------------------------------
    # VDA5050 message handlers
    # ------------------------------------------------------------------

    def _handle_order(self, payload: dict):
        order_id = payload.get('orderId', '')
        nodes = payload.get('nodes', [])

        self.get_logger().info(
            f"Received order '{order_id}' with {len(nodes)} nodes"
        )

        waypoints = []
        for node in sorted(nodes, key=lambda n: n['sequenceId']):
            pos = node.get('nodePosition', {})
            waypoints.append({
                'nodeId': node['nodeId'],
                'x': pos.get('x', self._x),
                'y': pos.get('y', self._y),
                'theta': pos.get('theta', self._theta),
            })

        with self._lock:
            self._order_id = order_id
            self._waypoint_queue = waypoints
            self._cancel_requested = False

        if self._motion_thread is None or not self._motion_thread.is_alive():
            self._motion_thread = threading.Thread(
                target=self._execute_waypoints, daemon=True
            )
            self._motion_thread.start()

    def _handle_instant_actions(self, payload: dict):
        for action in payload.get('instantActions', []):
            action_type = action.get('actionType', '')
            self.get_logger().info(f"Instant action: {action_type}")

            if action_type == 'cancelOrder':
                with self._lock:
                    self._waypoint_queue.clear()
                    self._cancel_requested = True
                    self._driving = False
                    handle = self._current_goal_handle

                if handle is not None:
                    self.get_logger().info("Cancelling current Nav2 goal")
                    handle.cancel_goal_async()

            elif action_type == 'factsheetRequest':
                self._publish_factsheet()

            elif action_type in ('startPause', 'stopPause'):
                self.get_logger().info(f"'{action_type}' not yet implemented")

    # ------------------------------------------------------------------
    # Nav2 motion
    # ------------------------------------------------------------------

    def _execute_waypoints(self):
        """Drive through the waypoint queue by sending Nav2 goals one at a time."""
        if not self._nav2_client.wait_for_server(
            timeout_sec=self._nav2_timeout
        ):
            self.get_logger().error(
                f"Nav2 action server '{self._action_server}' not available "
                f"after {self._nav2_timeout}s — order aborted"
            )
            with self._lock:
                self._errors.append({
                    'errorType': 'nav2Unavailable',
                    'errorDescription': 'Nav2 action server not found',
                    'errorLevel': 'FATAL',
                })
            return

        while True:
            with self._lock:
                if self._cancel_requested or not self._waypoint_queue:
                    self._driving = False
                    break
                target = self._waypoint_queue.pop(0)

            self.get_logger().info(
                f"Navigating to node '{target['nodeId']}' "
                f"({target['x']:.2f}, {target['y']:.2f}, "
                f"theta={math.degrees(target['theta']):.1f}°)"
            )

            with self._lock:
                self._driving = True

            success = self._send_nav2_goal(
                target['x'], target['y'], target['theta']
            )

            if not success:
                self.get_logger().warn(
                    f"Navigation to '{target['nodeId']}' failed or was cancelled"
                )
                with self._lock:
                    self._driving = False
                break

            with self._lock:
                self._last_node_id = target['nodeId']
                self.get_logger().info(
                    f"Reached node '{target['nodeId']}'"
                )

        with self._lock:
            order_id = self._order_id

        self.get_logger().info(f"Order '{order_id}' complete")

    def _send_nav2_goal(self, x: float, y: float, theta: float) -> bool:
        """
        Send a NavigateToPose goal and block until it succeeds or fails.
        Returns True on success, False on failure/cancellation.
        Uses threading.Event so the motion thread can block while the
        ROS2 spin thread processes action callbacks.
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self._map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        qz, qw = _quaternion_from_yaw(theta)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        done_event = threading.Event()
        result_holder = {'success': False}

        def goal_response_cb(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn("Nav2 goal rejected")
                done_event.set()
                return
            with self._lock:
                self._current_goal_handle = handle
            result_future = handle.get_result_async()
            result_future.add_done_callback(result_cb)

        def result_cb(future):
            from action_msgs.msg import GoalStatus
            status = future.result().status
            result_holder['success'] = (status == GoalStatus.STATUS_SUCCEEDED)
            with self._lock:
                self._current_goal_handle = None
            done_event.set()

        send_future = self._nav2_client.send_goal_async(goal_msg)
        send_future.add_done_callback(goal_response_cb)

        done_event.wait()
        return result_holder['success']

    # ------------------------------------------------------------------
    # VDA5050 state publishing
    # ------------------------------------------------------------------

    def _next_header(self) -> int:
        self._header_counter += 1
        return self._header_counter

    def _publish_factsheet(self):
        msg = {
            "headerId": self._next_header(),
            "timestamp": time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            "version": self._version,
            "manufacturer": self._manufacturer,
            "serialNumber": self._serial,
            "typeSpecification": {
                "seriesName": self._series_name,
                "agvKinematic": "DIFF",
                "agvClass": "CARRIER",
                "maxLoadMass": self._max_load_mass,
                "localizationTypes": ["NATURAL"],
                "navigationTypes": ["AUTONOMOUS"],
            },
            "physicalParameters": {
                "speedMin": 0.0,
                "speedMax": self._speed_max,
                "accelerationMax": self._accel_max,
                "decelerationMax": self._decel_max,
                "heightMin": 0.0,
                "heightMax": self._robot_height,
                "width": self._robot_width,
                "length": self._robot_length,
            },
            "protocolLimits": {
                "maxStringLens": {
                    "msgLen": 1024,
                    "topicSerialLen": 128,
                    "idLen": 128,
                    "idNumericalOnly": False,
                    "enumLen": 128,
                    "loadIdLen": 128,
                },
                "maxArrayLens": {
                    "order_nodes": 50,
                    "order_edges": 50,
                    "node_actions": 16,
                    "edge_actions": 16,
                    "instantActions": 16,
                    "state_nodeStates": 50,
                    "state_edgeStates": 50,
                    "state_loads": 16,
                    "state_actionStates": 32,
                    "state_errors": 32,
                    "state_information": 32,
                },
                "timing": {
                    "minOrderInterval": 0.5,
                    "minStateInterval": 0.5,
                    "defaultStateInterval": 1.0 / self._state_rate,
                    "visualizationInterval": 0.2,
                },
            },
            "protocolFeatures": {
                "optionalParameters": [],
                "agvActions": [
                    {
                        "actionType": "cancelOrder",
                        "actionScopes": ["INSTANT"],
                        "actionParameters": [],
                        "resultDescription": "Clears the active order and stops motion",
                    },
                    {
                        "actionType": "factsheetRequest",
                        "actionScopes": ["INSTANT"],
                        "actionParameters": [],
                        "resultDescription": "Re-publishes the factsheet",
                    },
                    {
                        "actionType": "startPause",
                        "actionScopes": ["INSTANT"],
                        "actionParameters": [],
                        "resultDescription": "Not yet implemented",
                    },
                    {
                        "actionType": "stopPause",
                        "actionScopes": ["INSTANT"],
                        "actionParameters": [],
                        "resultDescription": "Not yet implemented",
                    },
                ],
            },
        }
        self._mqtt.publish(self._topic('factsheet'), json.dumps(msg), qos=1, retain=True)
        self.get_logger().info("Factsheet published")

    def _publish_connection(self, state: str):
        msg = {
            "headerId": self._next_header(),
            "timestamp": time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            "version": self._version,
            "manufacturer": self._manufacturer,
            "serialNumber": self._serial,
            "connectionState": state,
        }
        self._mqtt.publish(self._topic('connection'), json.dumps(msg), qos=1)

    def _publish_state(self):
        with self._lock:
            x, y, theta = self._x, self._y, self._theta
            pose_init = self._pose_initialized
            driving = self._driving
            battery = self._battery
            order_id = self._order_id
            last_node = self._last_node_id
            errors = list(self._errors)

        msg = {
            "headerId": self._next_header(),
            "timestamp": time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            "version": self._version,
            "manufacturer": self._manufacturer,
            "serialNumber": self._serial,
            "orderId": order_id,
            "orderUpdateCounter": 0,
            "lastNodeId": last_node,
            "lastNodeSequenceId": 0,
            "driving": driving,
            "operatingMode": "AUTOMATIC",
            "agvPosition": {
                "x": round(x, 4),
                "y": round(y, 4),
                "theta": round(theta, 4),
                "mapId": "L1",
                "positionInitialized": pose_init,
            },
            "velocity": {
                "vx": 0.0,
                "vy": 0.0,
                "omega": 0.0,
            },
            "batteryState": {
                "batteryCharge": round(battery, 1),
                "charging": False,
            },
            "errors": errors,
            "nodeStates": [],
            "edgeStates": [],
            "actionStates": [],
            "loads": [],
            "safetyState": {
                "eStop": "NONE",
                "fieldViolation": False,
            },
        }
        self._mqtt.publish(self._topic('state'), json.dumps(msg), qos=0)

    def destroy_node(self):
        self._publish_connection('OFFLINE')
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        super().destroy_node()
