cmake_minimum_required(VERSION 3.8)
project(rmf_vda5050_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rmf_battery REQUIRED)

add_executable(adapter
  src/main.cpp
  src/VDA5050Adapter.cpp
  src/RobotHandle.cpp
)

target_include_directories(adapter PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(adapter
  rmf_fleet_adapter::rmf_fleet_adapter
  rmf_traffic::rmf_traffic
  nlohmann_json::nlohmann_json
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  yaml-cpp
  mosquitto
  rmf_battery::rmf_battery
)

install(TARGETS adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/dispatch_patrol.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dispatch_patrol
)

ament_package()
