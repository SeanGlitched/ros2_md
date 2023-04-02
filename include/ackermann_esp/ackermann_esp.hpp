#pragma once
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>

class MyHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;

  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;
};