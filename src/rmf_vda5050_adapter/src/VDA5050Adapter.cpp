#include <rmf_vda5050_adapter/VDA5050Adapter.hpp>

#include <rmf_fleet_adapter/agv/parse_graph.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/geometry/ConvexShape.hpp>
#include <rmf_traffic/Profile.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/MechanicalSystem.hpp>
#include <rmf_battery/agv/PowerSystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <stdexcept>
#include <sstream>

namespace rmf_vda5050_adapter {

// ---------------------------------------------------------------------------
VDA5050Adapter::VDA5050Adapter(const Config& config)
: _cfg(config)
, _logger(rclcpp::get_logger("VDA5050Adapter"))
{
  setup_rmf();

  // ---- MQTT ----------------------------------------------------------------
  mosquitto_lib_init();
  _mqtt = mosquitto_new(nullptr, true, this);
  if (!_mqtt)
    throw std::runtime_error("Failed to create mosquitto instance");

  mosquitto_connect_callback_set(_mqtt, on_connect_cb);
  mosquitto_message_callback_set(_mqtt, on_message_cb);
  mosquitto_disconnect_callback_set(_mqtt, on_disconnect_cb);

  if (!config.mqtt_username.empty())
    mosquitto_username_pw_set(
      _mqtt,
      config.mqtt_username.c_str(),
      config.mqtt_password.c_str());

  const int rc = mosquitto_connect(
    _mqtt, config.mqtt_host.c_str(), config.mqtt_port, 60);
  if (rc != MOSQ_ERR_SUCCESS)
  {
    throw std::runtime_error(
      std::string("MQTT connect failed: ") + mosquitto_strerror(rc));
  }

  // Non-blocking background loop
  mosquitto_loop_start(_mqtt);

  _adapter->start();

  RCLCPP_INFO(_logger,
    "VDA5050 adapter ready — fleet='%s', broker=%s:%d",
    _cfg.fleet_name.c_str(),
    _cfg.mqtt_host.c_str(),
    _cfg.mqtt_port);
}

// ---------------------------------------------------------------------------
VDA5050Adapter::~VDA5050Adapter()
{
  if (_adapter)
    _adapter->stop();

  if (_mqtt)
  {
    mosquitto_loop_stop(_mqtt, true);
    mosquitto_disconnect(_mqtt);
    mosquitto_destroy(_mqtt);
  }
  mosquitto_lib_cleanup();
}

// ---------------------------------------------------------------------------
// RMF setup
// ---------------------------------------------------------------------------

void VDA5050Adapter::setup_rmf()
{
  // Vehicle profile and traits must be built first — parse_graph needs them
  auto footprint = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(_cfg.robot_radius);
  auto vicinity  = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(_cfg.robot_radius * 1.5);
  const rmf_traffic::Profile profile{footprint, vicinity};

  rmf_traffic::agv::VehicleTraits traits{
    {_cfg.linear_velocity,  _cfg.linear_acceleration},
    {_cfg.angular_velocity, _cfg.angular_acceleration},
    profile
  };

  // Load nav graph from YAML (set by main.cpp — never empty at this point)
  try
  {
    _nav_graph = rmf_fleet_adapter::agv::parse_graph(
      _cfg.nav_graph_file, traits);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
      std::string("Failed to parse nav graph '") +
      _cfg.nav_graph_file + "': " + e.what());
  }

  // Production adapter — requires rmf_traffic_schedule to be running.
  // Uses Adapter::make() since rclcpp::init() is already called in main.cpp.
  _adapter = rmf_fleet_adapter::agv::Adapter::make(
    "rmf_vda5050_adapter",
    rclcpp::NodeOptions(),
    std::chrono::seconds(10));
  if (!_adapter)
    throw std::runtime_error(
      "Timed out waiting for rmf_traffic_schedule (10 s). "
      "Start it first: ros2 run rmf_traffic_ros2 rmf_traffic_schedule");

  _fleet = _adapter->add_fleet(_cfg.fleet_name, traits, _nav_graph);

  // Task planner parameters — required for the fleet to bid on tasks.
  auto battery_system = std::make_shared<rmf_battery::agv::BatterySystem>(
    *rmf_battery::agv::BatterySystem::make(
      _cfg.battery_voltage,
      _cfg.battery_capacity,
      _cfg.battery_charging_current));

  auto mechanical_system = std::make_shared<rmf_battery::agv::MechanicalSystem>(
    *rmf_battery::agv::MechanicalSystem::make(
      _cfg.robot_mass,
      /*moment_of_inertia=*/10.0,
      /*friction_coefficient=*/0.22));

  auto motion_sink = std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
    *battery_system, *mechanical_system);

  // Ambient (idle) and tool power draw — 20 W each
  auto ambient_power = *rmf_battery::agv::PowerSystem::make(20.0);
  auto ambient_sink  = std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, ambient_power);
  auto tool_sink     = std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, ambient_power);

  const bool ok = _fleet->set_task_planner_params(
    battery_system,
    motion_sink,
    ambient_sink,
    tool_sink,
    _cfg.recharge_threshold,
    _cfg.recharge_soc,
    /*account_for_battery_drain=*/false);

  if (!ok)
    RCLCPP_WARN(_logger, "set_task_planner_params failed — check battery values");

  // Accept patrol tasks (go-to-place); add more as needed
  _fleet->consider_patrol_requests(
    [logger = _logger](const nlohmann::json& desc,
      rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation& confirm)
    {
      RCLCPP_INFO(logger, "Patrol task received: %s", desc.dump().c_str());
      confirm.accept();
    });

  RCLCPP_INFO(_logger,
    "RMF fleet '%s' created with %zu waypoints",
    _cfg.fleet_name.c_str(),
    _nav_graph.num_waypoints());
}

// ---------------------------------------------------------------------------
// MQTT callbacks
// ---------------------------------------------------------------------------

void VDA5050Adapter::on_connect_cb(
  struct mosquitto* /*mosq*/, void* obj, int rc)
{
  static_cast<VDA5050Adapter*>(obj)->on_connect(rc);
}

void VDA5050Adapter::on_message_cb(
  struct mosquitto* /*mosq*/, void* obj,
  const struct mosquitto_message* msg)
{
  static_cast<VDA5050Adapter*>(obj)->on_message(msg);
}

void VDA5050Adapter::on_disconnect_cb(
  struct mosquitto* /*mosq*/, void* obj, int rc)
{
  static_cast<VDA5050Adapter*>(obj)->on_disconnect(rc);
}

void VDA5050Adapter::on_connect(int rc)
{
  if (rc != 0)
  {
    RCLCPP_ERROR(_logger, "MQTT connect refused, rc=%d", rc);
    return;
  }

  const std::string base = "uagv/" + _cfg.vda5050_version;
  mosquitto_subscribe(_mqtt, nullptr, (base + "/+/+/state").c_str(), 0);
  mosquitto_subscribe(_mqtt, nullptr, (base + "/+/+/connection").c_str(), 0);

  RCLCPP_INFO(_logger,
    "Connected to MQTT broker, subscribed to %s/+/+/{state,connection}",
    base.c_str());
}

void VDA5050Adapter::on_message(const struct mosquitto_message* msg)
{
  if (!msg->payloadlen)
    return;

  nlohmann::json payload;
  try
  {
    payload = nlohmann::json::parse(
      static_cast<const char*>(msg->payload),
      static_cast<const char*>(msg->payload) + msg->payloadlen);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(_logger, "Bad JSON on %s: %s", msg->topic, e.what());
    return;
  }

  // Parse topic: uagv/{version}/{manufacturer}/{serial}/{kind}
  std::string topic(msg->topic);
  std::istringstream ss(topic);
  std::string part;
  std::vector<std::string> parts;
  while (std::getline(ss, part, '/'))
    parts.push_back(part);

  if (parts.size() < 5)
    return;

  const std::string& manufacturer = parts[2];
  const std::string& serial       = parts[3];
  const std::string& kind         = parts[4];

  auto handle = get_or_create_robot(manufacturer, serial);

  if (kind == "state")
    handle_state_msg(handle, payload);
  else if (kind == "connection")
    handle_connection_msg(handle, payload);
}

void VDA5050Adapter::on_disconnect(int rc)
{
  RCLCPP_WARN(_logger, "MQTT disconnected (rc=%d), will reconnect", rc);
}

// ---------------------------------------------------------------------------
// Robot management
// ---------------------------------------------------------------------------

std::shared_ptr<RobotHandle> VDA5050Adapter::get_or_create_robot(
  const std::string& manufacturer,
  const std::string& serial)
{
  const std::string robot_id = manufacturer + "/" + serial;

  std::lock_guard<std::mutex> lock(_robots_mutex);

  auto it = _robots.find(robot_id);
  if (it != _robots.end())
    return it->second.second;

  // New robot discovered
  auto state = std::make_shared<RobotState>();
  state->manufacturer = manufacturer;
  state->serial       = serial;
  state->robot_id     = robot_id;

  auto handle = std::make_shared<RobotHandle>(
    state, _mqtt,
    _cfg.vda5050_version, _cfg.map_name,
    _cfg.linear_velocity,
    rclcpp::get_logger("RobotHandle." + robot_id));

  _robots[robot_id] = {state, handle};

  RCLCPP_INFO(_logger, "Discovered new robot: %s", robot_id.c_str());

  // Register with RMF fleet
  auto footprint = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(_cfg.robot_radius);
  const rmf_traffic::Profile profile{footprint};

  const auto now_ns = _adapter->node()->now().nanoseconds();
  const rmf_traffic::Time rmf_now{
    std::chrono::nanoseconds(now_ns)};

  rmf_traffic::agv::Plan::StartSet starts = {
    rmf_traffic::agv::Plan::Start(rmf_now, 0, 0.0)
  };

  _fleet->add_robot(
    handle,
    robot_id,
    profile,
    starts,
    [handle](std::shared_ptr<rmf_fleet_adapter::agv::RobotUpdateHandle> h)
    {
      handle->set_update_handle(std::move(h));
    });

  RCLCPP_INFO(_logger, "Robot '%s' registered with RMF", robot_id.c_str());

  return handle;
}

void VDA5050Adapter::handle_state_msg(
  const std::shared_ptr<RobotHandle>& handle,
  const nlohmann::json& payload)
{
  // We need the RobotState pointer — get it from _robots
  std::shared_ptr<RobotState> state;
  {
    std::lock_guard<std::mutex> lock(_robots_mutex);
    for (auto& [id, pair] : _robots)
    {
      if (pair.second == handle) { state = pair.first; break; }
    }
  }
  if (!state) return;

  if (payload.contains("agvPosition"))
  {
    const auto& pos = payload["agvPosition"];
    state->x     = pos.value("x",     state->x);
    state->y     = pos.value("y",     state->y);
    state->theta = pos.value("theta", state->theta);
  }

  state->driving      = payload.value("driving",      state->driving);
  state->order_id     = payload.value("orderId",      state->order_id);
  state->last_node_id = payload.value("lastNodeId",   state->last_node_id);

  if (payload.contains("batteryState"))
  {
    const double raw = payload["batteryState"].value(
      "batteryCharge", state->battery_soc * 100.0);
    state->battery_soc = raw / 100.0;
  }

  RCLCPP_DEBUG(_logger,
    "[%s] pos=(%.2f,%.2f) driving=%d batt=%.0f%%",
    state->robot_id.c_str(),
    state->x, state->y,
    static_cast<int>(state->driving),
    state->battery_soc * 100.0);

  handle->on_state_update();
}

void VDA5050Adapter::handle_connection_msg(
  const std::shared_ptr<RobotHandle>& handle,
  const nlohmann::json& payload)
{
  // Find state
  std::shared_ptr<RobotState> state;
  {
    std::lock_guard<std::mutex> lock(_robots_mutex);
    for (auto& [id, pair] : _robots)
    {
      if (pair.second == handle) { state = pair.first; break; }
    }
  }
  if (!state) return;

  const std::string conn_state =
    payload.value("connectionState", std::string("OFFLINE"));
  state->connected = (conn_state == "ONLINE");

  RCLCPP_INFO(_logger, "Robot %s connection: %s",
    state->robot_id.c_str(), conn_state.c_str());
}

}  // namespace rmf_vda5050_adapter
