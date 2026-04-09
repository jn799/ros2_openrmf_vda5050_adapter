"""
VDA5050 Mock Robot

Simulates a single VDA5050-compliant AGV for adapter development and testing.

Behaviour:
  - Publishes a 'connection' ONLINE message on startup.
  - Listens for 'order' messages from the adapter.
  - On receiving an order, "drives" node-by-node (linear interpolation, no physics).
  - Publishes 'state' messages at a configurable rate throughout.
  - Reports order completion when all nodes are reached.
"""

import json
import math
import threading
import time
import uuid

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node


class MockRobot(Node):

    def __init__(self):
        super().__init__('vda5050_mock_robot')

        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('manufacturer', 'mock')
        self.declare_parameter('serial_number', 'robot_01')
        self.declare_parameter('vda5050_version', 'v2')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_theta', 0.0)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('state_publish_rate', 1.0)

        self._host = self.get_parameter('mqtt_host').value
        self._port = self.get_parameter('mqtt_port').value
        self._user = self.get_parameter('mqtt_username').value
        self._pass = self.get_parameter('mqtt_password').value
        self._manufacturer = self.get_parameter('manufacturer').value
        self._serial = self.get_parameter('serial_number').value
        self._version = self.get_parameter('vda5050_version').value
        self._speed = self.get_parameter('linear_speed').value
        self._state_rate = self.get_parameter('state_publish_rate').value

        self._x = self.get_parameter('start_x').value
        self._y = self.get_parameter('start_y').value
        self._theta = self.get_parameter('start_theta').value

        self._driving = False
        self._battery = 100.0  # percent
        self._order_id = ''
        self._last_node_id = 'start'
        self._errors: list = []
        self._header_counter = 0
        self._lock = threading.Lock()

        # Queue of (x, y, theta, nodeId) waypoints to visit
        self._waypoint_queue: list[dict] = []
        self._motion_thread: threading.Thread | None = None

        self._setup_mqtt()

        interval = 1.0 / self._state_rate
        self.create_timer(interval, self._publish_state)

        self.get_logger().info(
            f"Mock robot started: {self._manufacturer}/{self._serial} "
            f"at ({self._x}, {self._y})"
        )

    # ------------------------------------------------------------------
    # MQTT
    # ------------------------------------------------------------------

    def _topic(self, kind: str) -> str:
        return f"uagv/{self._version}/{self._manufacturer}/{self._serial}/{kind}"

    def _setup_mqtt(self):
        self._mqtt = mqtt.Client(client_id=f"mock_{self._serial}_{uuid.uuid4().hex[:6]}")
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
        self.get_logger().info("Mock robot connected to MQTT broker")
        client.subscribe(self._topic('order'), qos=1)
        client.subscribe(self._topic('instantActions'), qos=1)
        self._publish_connection('ONLINE')

    def _on_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT disconnected (rc={rc})")

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Bad JSON: {e}")
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

        # Start motion in a background thread so MQTT loop stays responsive
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
                    self._driving = False
                self.get_logger().info("Order cancelled")
            elif action_type == 'startPause':
                self.get_logger().info("Pause requested (not yet implemented)")
            elif action_type == 'stopPause':
                self.get_logger().info("Resume requested (not yet implemented)")

    # ------------------------------------------------------------------
    # Simulated motion
    # ------------------------------------------------------------------

    def _execute_waypoints(self):
        while True:
            with self._lock:
                if not self._waypoint_queue:
                    self._driving = False
                    break
                target = self._waypoint_queue.pop(0)

            self.get_logger().info(
                f"Driving to node '{target['nodeId']}' "
                f"({target['x']:.2f}, {target['y']:.2f})"
            )
            self._drive_to(target['x'], target['y'], target['theta'])

            with self._lock:
                self._last_node_id = target['nodeId']

        self.get_logger().info(f"Order '{self._order_id}' complete")

    def _drive_to(self, tx: float, ty: float, t_theta: float):
        """Linearly interpolate position to (tx, ty) at self._speed m/s."""
        dt = 0.1  # simulation timestep seconds

        with self._lock:
            self._driving = True

        while True:
            with self._lock:
                cx, cy = self._x, self._y

            dx = tx - cx
            dy = ty - cy
            dist = math.hypot(dx, dy)

            if dist < 0.05:
                with self._lock:
                    self._x = tx
                    self._y = ty
                    self._theta = t_theta
                    self._driving = False
                break

            step = min(self._speed * dt, dist)
            with self._lock:
                self._x += (dx / dist) * step
                self._y += (dy / dist) * step
                self._theta = math.atan2(dy, dx)
                self._battery = max(0.0, self._battery - 0.001)

            time.sleep(dt)

    # ------------------------------------------------------------------
    # VDA5050 state publishing
    # ------------------------------------------------------------------

    def _next_header(self) -> int:
        self._header_counter += 1
        return self._header_counter

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
                "positionInitialized": True,
            },
            "velocity": {
                "vx": self._speed if driving else 0.0,
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
