#include <rmf_vda5050_adapter/RobotHandle.hpp>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <ctime>
#include <sstream>
#include <iomanip>

namespace rmf_vda5050_adapter {

// ---------------------------------------------------------------------------
static std::string utc_now()
{
  std::time_t t = std::time(nullptr);
  std::ostringstream ss;
  ss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%SZ");
  return ss.str();
}

// ---------------------------------------------------------------------------
RobotHandle::RobotHandle(
  std::shared_ptr<RobotState> state,
  struct mosquitto* mqtt,
  const std::string& vda_version,
  const std::string& map_name,
  double max_speed,
  rclcpp::Logger logger)
: _state(std::move(state))
, _mqtt(mqtt)
, _vda_version(vda_version)
, _map_name(map_name)
, _max_speed(max_speed)
, _logger(logger)
{}

// ---------------------------------------------------------------------------
void RobotHandle::set_update_handle(UpdateHandlePtr handle)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _update_handle = std::move(handle);
  RCLCPP_INFO(_logger, "RMF update handle ready for [%s]",
    _state->robot_id.c_str());
}

// ---------------------------------------------------------------------------
// RobotCommandHandle interface
// ---------------------------------------------------------------------------

void RobotHandle::follow_new_path(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
  ArrivalEst next_arrival_estimator,
  ReqCompleted path_finished_callback)
{
  std::lock_guard<std::mutex> lock(_mutex);

  _active_waypoints  = waypoints;
  _active_node_ids.clear();
  _current_wp_idx    = 0;
  _done_cb           = std::move(path_finished_callback);
  _arrival_est_cb    = std::move(next_arrival_estimator);

  send_order(waypoints);

  RCLCPP_INFO(_logger, "[%s] follow_new_path: %zu waypoints, order=%s",
    _state->robot_id.c_str(), waypoints.size(), _active_order_id.c_str());
}

void RobotHandle::stop()
{
  std::lock_guard<std::mutex> lock(_mutex);
  _done_cb = nullptr;
  _active_waypoints.clear();
  _active_node_ids.clear();
  send_instant_action("cancelOrder");
  RCLCPP_INFO(_logger, "[%s] stop", _state->robot_id.c_str());
}

void RobotHandle::dock(
  const std::string& dock_name,
  ReqCompleted docking_finished_callback)
{
  nlohmann::json params = {{"stationId", dock_name}};
  send_instant_action("startCharging", params);
  // Assume immediate completion — a production adapter would wait for
  // a docking-complete state from the robot before calling this.
  docking_finished_callback();
  RCLCPP_INFO(_logger, "[%s] dock at '%s'",
    _state->robot_id.c_str(), dock_name.c_str());
}

// ---------------------------------------------------------------------------
// State update
// ---------------------------------------------------------------------------

void RobotHandle::on_state_update()
{
  std::lock_guard<std::mutex> lock(_mutex);

  auto handle = _update_handle;
  if (!handle)
    return;

  handle->update_battery_soc(_state->battery_soc);

  if (_active_waypoints.empty())
  {
    // Idle — if robot is at a known graph node, report by waypoint index so
    // RMF knows it's at a holding point and doesn't re-dispatch immediately.
    const std::string& idle_node = _state->last_node_id;
    if (idle_node.size() > 1 && idle_node[0] == 'g')
    {
      try {
        const std::size_t wp_idx = std::stoul(idle_node.substr(1));
        handle->update_position(wp_idx, _state->theta);
        return;
      } catch (...) {}
    }
    handle->update_position(
      _map_name,
      Eigen::Vector3d{_state->x, _state->y, _state->theta});
    return;
  }

  // Advance tracking index to the waypoint matching last_node_id
  const std::string& last_node = _state->last_node_id;
  if (!last_node.empty())
  {
    for (std::size_t i = 0; i < _active_node_ids.size(); ++i)
    {
      if (_active_node_ids[i] == last_node && i > _current_wp_idx)
        _current_wp_idx = i;
    }
  }

  const auto& target = _active_waypoints[_current_wp_idx];
  const Eigen::Vector3d target_pos = target.position();
  const double dist = std::hypot(
    _state->x - target_pos.x(),
    _state->y - target_pos.y());

  if (dist < 0.3 && target.graph_index().has_value())
  {
    handle->update_position(*target.graph_index(), _state->theta);
  }
  else
  {
    const std::size_t next_wp =
      target.graph_index().value_or(0);
    handle->update_position(
      Eigen::Vector3d{_state->x, _state->y, _state->theta},
      next_wp);
  }

  // Tell RMF our ETA to the current target so it doesn't replan on timeout
  if (_arrival_est_cb)
  {
    const double secs = (_max_speed > 0.0) ? dist / _max_speed : 0.0;
    _arrival_est_cb(
      _current_wp_idx,
      std::chrono::duration_cast<rmf_traffic::Duration>(
        std::chrono::duration<double>(secs)));
  }

  // Signal path completion when robot has stopped at the final node AND has
  // acknowledged our specific order (prevents firing on stale pre-order state).
  const std::string& final_node =
    _active_node_ids.empty() ? "" : _active_node_ids.back();
  if (!_state->driving && last_node == final_node && _done_cb &&
      _state->order_id == _active_order_id)
  {
    RCLCPP_INFO(_logger, "[%s] path complete", _state->robot_id.c_str());
    _active_waypoints.clear();
    _active_node_ids.clear();
    auto cb = std::move(_done_cb);
    _done_cb = nullptr;
    // Release lock before calling cb to avoid potential deadlock
    _mutex.unlock();
    cb();
    _mutex.lock();
  }
}

// ---------------------------------------------------------------------------
// VDA5050 outbound helpers
// ---------------------------------------------------------------------------

void RobotHandle::send_order(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints)
{
  // Must be called with _mutex held
  _state->order_counter += 2;
  _active_order_id = next_order_id();
  _active_node_ids.clear();

  nlohmann::json nodes = nlohmann::json::array();
  nlohmann::json edges = nlohmann::json::array();

  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    const auto& wp  = waypoints[i];
    const auto  pos = wp.position();
    const auto  gidx = wp.graph_index();

    std::string node_id = gidx.has_value()
      ? ("g" + std::to_string(*gidx))
      : ("path_" + std::to_string(i));
    _active_node_ids.push_back(node_id);

    nodes.push_back({
      {"nodeId",     node_id},
      {"sequenceId", static_cast<int>(i * 2)},
      {"released",   true},
      {"nodePosition", {
        {"x",                    pos.x()},
        {"y",                    pos.y()},
        {"theta",                pos.z()},
        {"allowedDeviationXY",   0.5},
        {"allowedDeviationTheta", 0.3},
        {"mapId",                _map_name},
      }},
      {"actions", nlohmann::json::array()},
    });

    if (i + 1 < waypoints.size())
    {
      const std::string next_id = waypoints[i + 1].graph_index().has_value()
        ? ("g" + std::to_string(*waypoints[i + 1].graph_index()))
        : ("path_" + std::to_string(i + 1));

      edges.push_back({
        {"edgeId",      "e" + std::to_string(i)},
        {"sequenceId",  static_cast<int>(i * 2 + 1)},
        {"released",    true},
        {"startNodeId", node_id},
        {"endNodeId",   next_id},
        {"maxSpeed",    _max_speed},
        {"actions",     nlohmann::json::array()},
      });
    }
  }

  nlohmann::json order = {
    {"headerId",          _state->order_counter},
    {"timestamp",         utc_now()},
    {"version",           _vda_version},
    {"manufacturer",      _state->manufacturer},
    {"serialNumber",      _state->serial},
    {"orderId",           _active_order_id},
    {"orderUpdateCounter", _state->order_counter},
    {"nodes",             nodes},
    {"edges",             edges},
  };

  const std::string topic =
    "uagv/" + _vda_version + "/" +
    _state->manufacturer + "/" + _state->serial + "/order";
  const std::string payload = order.dump();

  mosquitto_publish(_mqtt, nullptr, topic.c_str(),
    static_cast<int>(payload.size()),
    payload.c_str(), 1, false);
}

void RobotHandle::send_instant_action(
  const std::string& action_type,
  const nlohmann::json& params)
{
  // May be called with or without _mutex held — caller manages locking
  _state->order_counter += 1;

  nlohmann::json action_params = nlohmann::json::array();
  if (!params.is_null())
  {
    for (auto it = params.begin(); it != params.end(); ++it)
      action_params.push_back({{"key", it.key()}, {"value", it.value()}});
  }

  nlohmann::json msg = {
    {"headerId",     _state->order_counter},
    {"timestamp",    utc_now()},
    {"version",      _vda_version},
    {"manufacturer", _state->manufacturer},
    {"serialNumber", _state->serial},
    {"instantActions", nlohmann::json::array({
      {
        {"actionId",        next_order_id()},
        {"actionType",      action_type},
        {"blockingType",    "HARD"},
        {"actionParameters", action_params},
      }
    })},
  };

  const std::string topic =
    "uagv/" + _vda_version + "/" +
    _state->manufacturer + "/" + _state->serial + "/instantActions";
  const std::string payload = msg.dump();

  mosquitto_publish(_mqtt, nullptr, topic.c_str(),
    static_cast<int>(payload.size()),
    payload.c_str(), 1, false);
}

std::string RobotHandle::next_order_id()
{
  // Simple counter-based ID — unique enough for a single-process adapter
  static std::atomic<uint64_t> counter{0};
  return "rmf_" + std::to_string(++counter);
}

}  // namespace rmf_vda5050_adapter
