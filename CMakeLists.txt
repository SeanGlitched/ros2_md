cmake_minimum_required(VERSION 3.8)
project(ros2_md)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# code dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motor_interfaces REQUIRED)
#msg and srv related
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# further executables manually.
# add_executable(<node_name> src/<code_name>.cpp)



# add executables
add_executable(my_node src/my_node.cpp)
add_executable(node2 src/node2.cpp)

add_executable(serial_node src/serial_node.cpp)
add_executable(cl_input src/cl_input.cpp)


add_executable(motor_comm src/motor_comm.cpp)

# add_executable(<node_name> src/<code_name>.cpp)


# Link the executable with the ROS2 libraries
ament_target_dependencies(motor_comm rclcpp std_msgs motor_interfaces)
ament_target_dependencies(serial_node rclcpp std_msgs)
ament_target_dependencies(cl_input rclcpp)

# custom msg and srv related
# find_package(<dependency> REQUIRED)



install(TARGETS 
  my_node
  node2
  serial_node
  cl_input
  motor_comm
  DESTINATION lib/${PROJECT_NAME}
)






ament_package()
