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


# add executables
add_executable(my_node src/my_node.cpp)
add_executable(node2 src/node2.cpp)

add_executable(serial_node src/serial_node.cpp)
add_executable(cl_input src/cl_input.cpp)


add_executable(motor_comm src/motor_comm.cpp)

# add_executable(<node_name> src/<code_name>.cpp)


# Link the executable with the ROS2 libraries
ament_target_dependencies(motor_comm rclcpp std_msgs)
ament_target_dependencies(serial_node rclcpp std_msgs)
ament_target_dependencies(cl_input rclcpp)

# custom msg and srv related
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/MotorCommand.msg")
set(srv_files
  "srv/ReadEncoder.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}  
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(motor_comm "${cpp_typesupport_target}")



install(TARGETS 
  my_node
  node2
  serial_node
  cl_input
  motor_comm
  DESTINATION lib/${PROJECT_NAME}
)



install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)




ament_package()