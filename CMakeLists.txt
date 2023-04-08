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
# # find_package(std_msgs REQUIRED)
# find_package(motor_interfaces REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)



# find_package(ros2_control REQUIRED)

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



add_library(${PROJECT_NAME} 
SHARED 
src/motor.cpp 
src/servo.cpp
src/esp_comms.cpp
src/ackermann_esp.cpp 
${serial_SRCS}
)


target_include_directories(
  ${PROJECT_NAME} 
  PRIVATE
  include
)


ament_target_dependencies(
  ${PROJECT_NAME} 
  hardware_interface
  rclcpp
  pluginlib
)


pluginlib_export_plugin_description_file(hardware_interface ackermann_esp.xml)


#install
install( 
  TARGETS ${PROJECT_NAME} 
  DESTINATION lib
)

install ( 
 DIRECTORY     include/
 DESTINATION   include    
 )




## EXPORTS
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}   
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

  ament_package()
  