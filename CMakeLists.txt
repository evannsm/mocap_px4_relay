cmake_minimum_required(VERSION 3.8)
project(mocap_px4_relay)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  px4_msgs
  geometry_msgs
  mocap4r2_msgs
  Eigen3
)

add_executable(optitrack_px4_relay
  src/optitrack_to_px4_relay.cpp
)
add_executable(full_state_relay
  src/full_state_relay.cpp
)

# (optional) if you plan to use vicon later
# add_executable(vicon_px4_relay src/vicon_px4_relay.cpp)
# ament_target_dependencies(vicon_px4_relay ${dependencies})

ament_target_dependencies(optitrack_px4_relay ${dependencies})
ament_target_dependencies(full_state_relay ${dependencies})


install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install ALL executables you want to launch
install(TARGETS
  optitrack_px4_relay
  full_state_relay          # <-- add this
  # vicon_px4_relay         # <-- add if you enable it
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()