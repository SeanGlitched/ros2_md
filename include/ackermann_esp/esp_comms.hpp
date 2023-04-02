#pragma once 

#include "serial/serial.h"


class ESP_Comms{

public: 

  ESP_Comms(){}

  ESP_Comms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    : serial_conn_(serial_device, baud_rate, timeout_ms)
  {  }


  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  
  void sendEmptyMsg();
  void readEncoderValues(int &val_1, int &val_2);
  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return serial_conn_.isOpen(); }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);

private:

  serial::Serial serial_conn_;
};