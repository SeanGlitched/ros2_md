cmake_minimum_required(VERSION 3.8)
project(ros2_md)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

##########################################################################
##########################################################################
##########################################################################
#SERIAL_LIBRARY


if(UNIX AND NOT APPLE)
    # If Linux, add rt and pthread
    set(rt_LIBRARIES rt)
    set(pthread_LIBRARIES pthread)
endif()
## Sources
set(serial_SRCS
    src/serial/serial.cc
)
if(APPLE)
    # If OSX
    list(APPEND serial_SRCS src/serial/impl/unix.cc)
    list(APPEND serial_SRCS src/serial/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS src/serial/impl/unix.cc)
    list(APPEND serial_SRCS src/serial/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS src/serial/impl/win.cc)
    list(APPEND serial_SRCS src/serial/impl/list_ports/list_ports_win.cc)
endif()

# ### Add serial library
# add_library(${PROJECT_NAME} ${serial_SRCS})
# #link library
# if(APPLE)
#     target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
# elseif(UNIX)
#     target_link_libraries(${PROJECT_NAME} rt pthread)
# else()
#     target_link_libraries(${PROJECT_NAME} setupapi)
# endif()


## Include headers
include_directories(include)




## Install headers
install (
        DIRECTORY include/
        DESTINATION include
)

# ament_export_include_directories(
#   include
# )

# ament_export_dependencies(rosidl_default_runtime)

# ament_package()








##########################################################################
##########################################################################
##########################################################################




# code dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motor_interfaces REQUIRED)

#find_package(serial REQUIRED)

#serial library https://github.com/wjwwood/serial

#message(STATUS "serial_INCLUDE_DIR: ${serial_INCLUDE_DIR}")

#msg and srv related
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# further executables manually.
# add_executable(<node_name> src/<code_name>.cpp)
#add header files
#include_directories(include)

# add executables
add_executable(my_node src/my_node.cpp)
add_executable(node2 src/node2.cpp)

add_executable(serial_node src/serial_node.cpp)
add_executable(cl_input src/cl_input.cpp)



add_executable(ros_serial_node 
src/ros_serial_node.cpp 
${serial_SRCS}
#src/serial/serial.cc)
) 
# add_executable(<node_name> src/<code_name>.cpp)


# Link the executable with the ROS2 libraries
#ament_target_dependencies(motor_comm rclcpp std_msgs motor_interfaces)
ament_target_dependencies(serial_node rclcpp std_msgs)
ament_target_dependencies(cl_input rclcpp)

ament_target_dependencies(ros_serial_node rclcpp std_msgs )#serial)
#add_executable(ros2_md src/ros2_md.cpp)

#link_directories(${serial_LIBRARY_DIRS})

#target_link_libraries(ros_serial_node ${serial_LIBRARIES})
# custom msg and srv related
# find_package(<dependency> REQUIRED)

install(TARGETS 
my_node
node2
serial_node
cl_input
ros_serial_node
DESTINATION lib/${PROJECT_NAME}
)

#install ( 
#  DIRECTORY     include/
#  DESTINATION   include    
#  )
  
  ament_package()
  