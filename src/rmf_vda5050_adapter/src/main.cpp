#include <rmf_vda5050_adapter/VDA5050Adapter.hpp>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <csignal>
#include <atomic>
#include <filesystem>
#include <thread>
#include <chrono>

namespace {
std::atomic<bool> g_shutdown{false};
void sig_handler(int) { g_shutdown = true; }
}

static rmf_vda5050_adapter::VDA5050Adapter::Config load_config(
  const std::string& path)
{
  rmf_vda5050_adapter::VDA5050Adapter::Config cfg;

  YAML::Node y = YAML::LoadFile(path);

  if (auto n = y["mqtt"])
  {
    cfg.mqtt_host     = n["host"].as<std::string>(cfg.mqtt_host);
    cfg.mqtt_port     = n["port"].as<int>(cfg.mqtt_port);
    cfg.mqtt_username = n["username"].as<std::string>(cfg.mqtt_username);
    cfg.mqtt_password = n["password"].as<std::string>(cfg.mqtt_password);
  }
  if (auto n = y["vda5050"])
    cfg.vda5050_version = n["version"].as<std::string>(cfg.vda5050_version);

  if (auto n = y["fleet"])
  {
    cfg.fleet_name     = n["name"].as<std::string>(cfg.fleet_name);
    cfg.map_name       = n["map_name"].as<std::string>(cfg.map_name);
    cfg.nav_graph_file = n["nav_graph_file"].as<std::string>(cfg.nav_graph_file);
  }
  if (auto n = y["robot"])
  {
    cfg.robot_radius         = n["radius"].as<double>(cfg.robot_radius);
    cfg.linear_velocity      = n["linear_velocity"].as<double>(cfg.linear_velocity);
    cfg.linear_acceleration  = n["linear_acceleration"].as<double>(cfg.linear_acceleration);
    cfg.angular_velocity     = n["angular_velocity"].as<double>(cfg.angular_velocity);
    cfg.angular_acceleration = n["angular_acceleration"].as<double>(cfg.angular_acceleration);
    cfg.robot_mass           = n["mass"].as<double>(cfg.robot_mass);
  }
  if (auto n = y["battery"])
  {
    cfg.battery_voltage          = n["voltage"].as<double>(cfg.battery_voltage);
    cfg.battery_capacity         = n["capacity_ah"].as<double>(cfg.battery_capacity);
    cfg.battery_charging_current = n["charging_current"].as<double>(cfg.battery_charging_current);
    cfg.recharge_threshold       = n["recharge_threshold"].as<double>(cfg.recharge_threshold);
    cfg.recharge_soc             = n["recharge_soc"].as<double>(cfg.recharge_soc);
  }

  return cfg;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::signal(SIGINT,  sig_handler);
  std::signal(SIGTERM, sig_handler);

  // Config file: first CLI arg, else package default
  std::string config_path;
  if (argc > 1 && std::filesystem::exists(argv[1]))
  {
    config_path = argv[1];
  }
  else
  {
    config_path = ament_index_cpp::get_package_share_directory(
      "rmf_vda5050_adapter") + "/config/adapter.yaml";
  }

  RCLCPP_INFO(rclcpp::get_logger("main"), "Loading config: %s",
    config_path.c_str());

  rmf_vda5050_adapter::VDA5050Adapter::Config cfg;
  try
  {
    cfg = load_config(config_path);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(rclcpp::get_logger("main"),
      "Failed to load config (%s), using defaults", e.what());
  }

  // Resolve nav_graph_file: empty → package default
  if (cfg.nav_graph_file.empty())
  {
    cfg.nav_graph_file =
      ament_index_cpp::get_package_share_directory("rmf_vda5050_adapter") +
      "/config/nav_graph.yaml";
  }
  RCLCPP_INFO(rclcpp::get_logger("main"), "Nav graph: %s",
    cfg.nav_graph_file.c_str());

  std::unique_ptr<rmf_vda5050_adapter::VDA5050Adapter> adapter;
  try
  {
    adapter = std::make_unique<rmf_vda5050_adapter::VDA5050Adapter>(cfg);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("main"),
      "Failed to start adapter: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("main"), "Adapter running — Ctrl+C to stop");

  while (!g_shutdown && rclcpp::ok())
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  adapter.reset();
  rclcpp::shutdown();
  return 0;
}
