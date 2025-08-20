cmake_minimum_required(VERSION 3.8)
project(simple_pure_pursuit)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(tf2 REQUIRED)
find_package(tier4_autoware_utils REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(simple_pure_pursuit
  src/simple_pure_pursuit.cpp
)

# tf2とtier4_autoware_utilsを明示的にリンク
target_link_libraries(simple_pure_pursuit
  tf2::tf2
)

ament_auto_package()
