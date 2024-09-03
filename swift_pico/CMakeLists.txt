cmake_minimum_required(VERSION 3.8)
project(swift_pico)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(swift_msgs REQUIRED)
find_package(pid_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(pico_controller_cpp src/pico_controller.cpp)
add_executable(map_cpp scripts/map.cpp)
ament_target_dependencies(pico_controller_cpp rclcpp swift_msgs pid_msg geometry_msgs)
ament_target_dependencies(map_cpp OpenCV)


install(TARGETS
  pico_controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  map_cpp
  DESTINATION lib/${PROJECT_NAME}
)

#Install scripts
install(PROGRAMS
  src/pico_controller.py
  scripts/map.py
  DESTINATION lib/${PROJECT_NAME}
)

#Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
