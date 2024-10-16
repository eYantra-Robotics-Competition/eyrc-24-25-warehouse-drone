cmake_minimum_required(VERSION 3.8)
project(swift_pico)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(swift_msgs REQUIRED)
find_package(waypoint_navigation REQUIRED)
find_package(pid_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(swift_pico_description REQUIRED)
find_package(rotors_swift_gazebo)


#executables

#task1a
add_executable(pico_controller_cpp src/pico_controller.cpp)
add_executable(map_cpp scripts/map.cpp)

#task2a
add_executable(pico_server_cpp src/pico_server.cpp)
add_executable(pico_client_cpp src/pico_client.cpp)
add_executable(waypoint_service_cpp src/waypoint_service.cpp)

#task2b
add_executable(pico_server_2b_cpp src/pico_server_2b.cpp)
add_executable(pico_client_2b_cpp src/pico_client_2b.cpp)
add_executable(path_planning_service_cpp src/path_planning_service.cpp)
add_executable(bit_map_cpp scripts/bit_map.cpp)

#task2c
add_executable(pico_server_2c_cpp src/pico_server_2c.cpp)
add_executable(pico_client_2c_cpp src/pico_client_2c.cpp)
add_executable(warehouse_service_cpp src/warehouse_service.cpp)

#Dependencies 

#task1a
ament_target_dependencies(pico_controller_cpp rclcpp swift_msgs pid_msg geometry_msgs)
ament_target_dependencies(map_cpp OpenCV)


#task2a
ament_target_dependencies(pico_server_cpp rclcpp swift_msgs pid_msg geometry_msgs waypoint_navigation)
ament_target_dependencies(pico_client_cpp rclcpp swift_msgs pid_msg geometry_msgs waypoint_navigation)
ament_target_dependencies(waypoint_service_cpp rclcpp swift_msgs pid_msg geometry_msgs waypoint_navigation)


#task2b
ament_target_dependencies(pico_server_2b_cpp rclcpp swift_msgs pid_msg geometry_msgs waypoint_navigation)
ament_target_dependencies(pico_client_2b_cpp rclcpp swift_msgs pid_msg geometry_msgs waypoint_navigation)
ament_target_dependencies(bit_map_cpp OpenCV)
ament_target_dependencies(path_planning_service_cpp rclcpp swift_msgs pid_msg geometry_msgs waypoint_navigation)


#task2c
ament_target_dependencies(pico_server_2c_cpp rclcpp swift_msgs pid_msg geometry_msgs waypoint_navigation)
ament_target_dependencies(pico_client_2c_cpp rclcpp swift_msgs pid_msg geometry_msgs waypoint_navigation)
ament_target_dependencies(warehouse_service_cpp rclcpp swift_msgs pid_msg geometry_msgs waypoint_navigation)


#Target installations

#task1

install(TARGETS
  pico_controller_cpp
  map_cpp
  DESTINATION lib/${PROJECT_NAME}
)

#task2
install(TARGETS
  waypoint_service_cpp
  pico_client_cpp
  pico_server_cpp
  path_planning_service_cpp
  pico_client_2b_cpp
  pico_server_2b_cpp
  bit_map_cpp
  warehouse_service_cpp
  pico_client_2c_cpp
  pico_server_2c_cpp
  DESTINATION lib/${PROJECT_NAME}
)

#Install scripts
install(PROGRAMS
  src/pico_controller.py
  scripts/map.py
  src/pico_client.py
  src/pico_server.py
  src/waypoint_service.py
  scripts/bit_map.py
  src/pico_client_2b.py
  src/pico_server_2b.py
  src/path_planning_service.py
  src/pico_client_2c.py
  src/pico_server_2c.py
  src/warehouse_service.py
  DESTINATION lib/${PROJECT_NAME}
)

#Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
    PROGRAMS
    wd_executables/planner_server
    wd_executables/warehouse_server
    DESTINATION lib/${PROJECT_NAME}  # This installs to install/lib/swift_pico
    PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

ament_package()
