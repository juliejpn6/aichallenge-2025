cmake_minimum_required(VERSION 3.8)
project(control_analysis_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存関係
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# PlotJuggler用データ配信システム（唯一残る機能）
add_executable(plotjuggler_data_publisher
  src/plotjuggler_data_publisher.cpp
)

target_include_directories(plotjuggler_data_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(plotjuggler_data_publisher
  rclcpp
  std_msgs
  autoware_auto_control_msgs
  tier4_vehicle_msgs
  autoware_auto_vehicle_msgs
  nav_msgs
)

# インストール設定（plotjuggler_data_publisherのみ）
install(TARGETS
  plotjuggler_data_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
