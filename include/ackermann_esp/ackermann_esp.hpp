#ifndef ACKERMAN_ESP_HPP
#define ACKERMAN_ESP_HPP




#include <cstring>
#include "rclcpp/rclcpp.hpp"

//from the tutorial
//#include "hardware_interface/system_interface.hpp"
//#include "hardware_interface/handle.hpp"
//#include "hardware_interface/hardware_info.hpp"
//#include "hardware_interface/types/hardware_interface_return_values.hpp"
//from the docs

//#include "hardware_interface/actuator_interface.hpp"
// #include "hardware_interface/system_interface.hpp"
// #include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


#include "config.hpp"
#include "ackermann_esp/esp_comms.hpp"
#include "ackermann_esp/motor.hpp"
#include "ackermann_esp/servo.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros2_md_h_i{

class AckermannESP : public hardware_interface::SystemInterface
{


public:


public:

  RCLCPP_SHARED_PTR_DEFINITIONS(AckermannESP);

  AckermannESP();

  //SystemInterface
  
  CallbackReturn 
  on_init(const hardware_interface::HardwareInfo & info) override;

  //LifeCycleNode
  CallbackReturn 
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn 
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn 
  on_activate(const rclcpp_lifecycle::State & previous_state) override;
    
  CallbackReturn 
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  CallbackReturn 
  on_error(const rclcpp_lifecycle::State & previous_state) override;
  
  std::vector<hardware_interface::StateInterface> 
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> 
  export_command_interfaces() override;

  hardware_interface::return_type 
  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type 
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;



private:

  ESPComms esp_;

  Motor motor_;

  Servo servo_;

  Config cfg_;

  rclcpp::Logger logger_;

  std::chrono::time_point<std::chrono::system_clock> time_;

  
};

}//ros2_md


#endif