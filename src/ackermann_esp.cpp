#include "ackermann_esp/ackermann_esp.hpp"


#include "hardware_interface/types/hardware_interface_type_values.hpp"



namespace ros2_md_h_i{



//using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using return_type = hardware_interface::return_type;

AckermannESP::AckermannESP()
    : logger_(rclcpp::get_logger("AckermannESP"))
{}


CallbackReturn AckermannESP::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  
  time_ = std::chrono::system_clock::now();

  cfg_.motor_name = info_.hardware_parameters["motor_name"]; 
  cfg_.servo_name = info_.hardware_parameters["servo_name"];
  //cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  

  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannESP::on_configure(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(logger_, "Configuring...");

  // Set up the wheels
  // l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  // r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Set up the wheel
  motor_.setup(cfg_.motor_name, cfg_.enc_counts_per_rev);
  
  //
  servo_.setup(cfg_.servo_name);

  // Set up the Arduino
  esp_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  

  RCLCPP_INFO(logger_, "Finished Configuration");
  return CallbackReturn::SUCCESS; 

}

CallbackReturn AckermannESP::on_cleanup(const rclcpp_lifecycle::State & previous_state){
  return CallbackReturn::SUCCESS; 
}

CallbackReturn AckermannESP::on_shutdown(const rclcpp_lifecycle::State & previous_state){
  RCLCPP_INFO(logger_, "Stopping Controller...");
  

//  return return_type::OK;
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannESP::on_activate(const rclcpp_lifecycle::State & previous_state){
  RCLCPP_INFO(logger_, "Starting Controller...");

  // esp_.sendEmptyMsg();
  // // arduino.setPidValues(9,7,0,100);
  // // arduino.setPidValues(14,7,0,100);
  // esp_.setPidValues(30, 20, 0, 100);

  return CallbackReturn::SUCCESS;
}
  
CallbackReturn AckermannESP::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "Stopping Controller...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannESP::on_error(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "Error Occurred...");

  return CallbackReturn::SUCCESS;
}


//add IMU
std::vector<hardware_interface::StateInterface> AckermannESP::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(motor_.name, hardware_interface::HW_IF_VELOCITY, &motor_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(motor_.name, hardware_interface::HW_IF_POSITION, &motor_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(servo_.name, hardware_interface::HW_IF_POSITION, &servo_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AckermannESP::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(motor_.name, hardware_interface::HW_IF_VELOCITY, &motor_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(servo_.name, hardware_interface::HW_IF_POSITION, &servo_.cmd));

  return command_interfaces;
}

hardware_interface::return_type AckermannESP::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{

  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;


  if (!esp_.connected())
  {
    return return_type::ERROR;
  }

  esp_.readEncoderValue(motor_.enc);

  double pos_prev = motor_.pos;
  motor_.pos = motor_.calcEncAngle();
  motor_.vel = (motor_.pos - pos_prev) / deltaSeconds;

  return return_type::OK;  
}

hardware_interface::return_type AckermannESP::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{

  if (!esp_.connected())
  {
    return return_type::ERROR;
  }

  //esp_.setMotorValue(l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate, r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate);

  esp_.setMotorValue(motor_.cmd, servo_.cmd);


  return return_type::OK;

}


}//namespace ros2_md

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_md_h_i::AckermannESP,
  hardware_interface::SystemInterface
)