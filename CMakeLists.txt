cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
# code dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#msg and srv related
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# add executables
add_executable(my_node src/my_node.cpp)
add_executable(node2 src/node2.cpp)

add_executable(serial src/serial.cpp)
add_executable(cl_input src/cl_input.cpp)

# further executables manually.
# add_executable(<node_name> src/<code_name>.cpp)


# Link the executable with the ROS2 libraries
ament_target_dependencies(serial rclcpp std_msgs)
ament_target_dependencies(cl_input rclcpp)

# custom msg and srv related

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MBCommand.msg"
 # DEPENDENCIES  # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)



install(TARGETS 
  my_node
  node2
  serial
  cl_input
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
