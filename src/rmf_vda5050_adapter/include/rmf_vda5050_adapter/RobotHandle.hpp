#pragma once

#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <mosquitto.h>
#include <nlohmann/json.hpp>

#include <rclcpp/logger.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <functional>
#include <atomic>

namespace rmf_vda5050_adapter {

// ---------------------------------------------------------------------------
// RobotState — plain data bag updated by incoming VDA5050 state messages
// ---------------------------------------------------------------------------
struct RobotState
{
  std::string manufacturer;
  std::string serial;
  std::string robot_id;   // "manufacturer/serial"

  double x{0.0};
  double y{0.0};
  double theta{0.0};
  bool driving{false};
  double battery_soc{1.0};  // 0.0 – 1.0
  std::string order_id;
  std::string last_node_id;
  bool connected{false};
  int order_counter{0};
};

// ---------------------------------------------------------------------------
// RobotHandle — RobotCommandHandle implementation
// Receives navigation commands from RMF and sends VDA5050 orders over MQTT.
// ---------------------------------------------------------------------------
class RobotHandle : public rmf_fleet_adapter::agv::RobotCommandHandle
{
public:
  using Duration       = rmf_traffic::Duration;
  using ArrivalEst     = ArrivalEstimator;
  using ReqCompleted   = RequestCompleted;
  using UpdateHandlePtr =
    std::shared_ptr<rmf_fleet_adapter::agv::RobotUpdateHandle>;

  RobotHandle(
    std::shared_ptr<RobotState> state,
    struct mosquitto* mqtt,
    const std::string& vda_version,
    const std::string& map_name,
    double max_speed,
    rclcpp::Logger logger);

  // ---- RobotCommandHandle interface ----------------------------------------

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEst next_arrival_estimator,
    ReqCompleted path_finished_callback) override;

  void stop() override;

  void dock(
    const std::string& dock_name,
    ReqCompleted docking_finished_callback) override;

  // ---- Called by VDA5050Adapter on every incoming state message -------------
  void on_state_update();

  void set_update_handle(UpdateHandlePtr handle);
  void set_online(bool online);

private:
  void send_order(const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints);
  void send_instant_action(const std::string& action_type,
    const nlohmann::json& params = nullptr);
  std::string next_order_id();

  std::shared_ptr<RobotState> _state;
  struct mosquitto* _mqtt;
  std::string _vda_version;
  std::string _map_name;
  double _max_speed;
  rclcpp::Logger _logger;

  mutable std::mutex _mutex;
  UpdateHandlePtr _update_handle;

  // Active path tracking
  std::vector<rmf_traffic::agv::Plan::Waypoint> _active_waypoints;
  std::vector<std::string> _active_node_ids;
  std::size_t _current_wp_idx{0};
  ReqCompleted _done_cb;
  ArrivalEst   _arrival_est_cb;
  std::string  _active_order_id;
};

}  // namespace rmf_vda5050_adapter
