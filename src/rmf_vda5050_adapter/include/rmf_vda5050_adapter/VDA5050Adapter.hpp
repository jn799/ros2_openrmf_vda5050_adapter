#pragma once

#include <rmf_vda5050_adapter/RobotHandle.hpp>

#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/Profile.hpp>

#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace rmf_vda5050_adapter {

class VDA5050Adapter
{
public:
  struct Config
  {
    // MQTT
    std::string mqtt_host     = "localhost";
    int         mqtt_port     = 1883;
    std::string mqtt_username;
    std::string mqtt_password;

    // VDA5050
    std::string vda5050_version = "v2";

    // RMF fleet
    std::string fleet_name    = "vda5050_fleet";
    std::string map_name      = "L1";
    std::string nav_graph_file;  // empty = use package default

    // Robot profile
    double robot_radius         = 0.3;
    double linear_velocity      = 0.5;
    double linear_acceleration  = 0.3;
    double angular_velocity     = 0.6;
    double angular_acceleration = 0.45;

    // Battery / task planner
    double battery_voltage          = 24.0;  // V
    double battery_capacity         = 40.0;  // Ah
    double battery_charging_current = 8.0;   // A
    double robot_mass               = 20.0;  // kg
    double recharge_threshold       = 0.2;   // recharge below 20% SOC
    double recharge_soc             = 1.0;   // charge back to 100%
  };

  explicit VDA5050Adapter(const Config& config);
  ~VDA5050Adapter();

  // Non-copyable / non-movable
  VDA5050Adapter(const VDA5050Adapter&) = delete;
  VDA5050Adapter& operator=(const VDA5050Adapter&) = delete;

private:
  // ---- RMF setup -----------------------------------------------------------
  void setup_rmf();

  // ---- MQTT callbacks (static → forward to instance methods) ---------------
  static void on_connect_cb(
    struct mosquitto* mosq, void* obj, int rc);
  static void on_message_cb(
    struct mosquitto* mosq, void* obj,
    const struct mosquitto_message* msg);
  static void on_disconnect_cb(
    struct mosquitto* mosq, void* obj, int rc);

  void on_connect(int rc);
  void on_message(const struct mosquitto_message* msg);
  void on_disconnect(int rc);

  // ---- Robot management ----------------------------------------------------
  std::shared_ptr<RobotHandle> get_or_create_robot(
    const std::string& manufacturer,
    const std::string& serial);

  void handle_state_msg(
    const std::shared_ptr<RobotHandle>& handle,
    const nlohmann::json& payload);

  void handle_connection_msg(
    const std::shared_ptr<RobotHandle>& handle,
    const nlohmann::json& payload);

  // ---- Data ----------------------------------------------------------------
  Config _cfg;
  rclcpp::Logger _logger;

  std::shared_ptr<rmf_fleet_adapter::agv::Adapter>  _adapter;
  std::shared_ptr<rmf_fleet_adapter::agv::FleetUpdateHandle>  _fleet;
  rmf_traffic::agv::Graph                                     _nav_graph;

  // MQTT
  struct mosquitto* _mqtt{nullptr};

  // Robots
  std::mutex _robots_mutex;
  std::unordered_map<
    std::string,
    std::pair<
      std::shared_ptr<RobotState>,
      std::shared_ptr<RobotHandle>>> _robots;
};

}  // namespace rmf_vda5050_adapter
