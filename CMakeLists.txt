cmake_minimum_required(VERSION 3.5)
project(optitrack2_px4_relay)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap_msgs REQUIRED)

set(dependencies
  rclcpp
  px4_msgs
  mocap_msgs
)

add_executable(optitrack2_px4_relay
  src/optitrack2_px4_relay.cpp
)

ament_target_dependencies(optitrack2_px4_relay ${dependencies})

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS optitrack2_px4_relay
        RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
