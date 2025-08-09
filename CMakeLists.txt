cmake_minimum_required(VERSION 3.8)
project(mocap_px4_relay)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mocap4r2_msgs REQUIRED)

set(dependencies
  rclcpp
  px4_msgs
  Eigen3
  mocap4r2_msgs
)

add_executable(optitrack_px4_relay
  src/optitrack_to_px4_relay.cpp
)

ament_target_dependencies(optitrack_px4_relay ${dependencies})

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS optitrack_px4_relay
        RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
