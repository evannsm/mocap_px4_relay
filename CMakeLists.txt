cmake_minimum_required(VERSION 3.8)
project(mocap_px4_relay)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vicon_receiver REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  px4_msgs
  vicon_receiver
  px4_ros_com
  Eigen3
)

add_executable(vicon_px4_relay
  src/vicon_px4_relay.cpp
)

ament_target_dependencies(vicon_px4_relay ${dependencies})

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS vicon_px4_relay
        RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
