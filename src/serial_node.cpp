#include <cstdio>
#include <iostream>
#include <string>

//Allows use of most common systems is ROS2
#include "rclcpp/rclcpp.hpp"
//Allows you the build in message type to publish data
#include "std_msgs/msg/string.hpp"

//serial related
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

<<<<<<< HEAD:src/serial_node.cpp
//#include "ros2_md/msg/mbcommand.hpp"
//#include "ros2_md/srv/encoder.hpp"

=======
#include "motor_interfaces/msg/motor_command.hpp"
>>>>>>> 2a9b443 (dual package):src/serial.cpp
#define DEBUG 0

using namespace std::chrono_literals;


//inherit from the Node::cpp class
class SerialCommunication : public rclcpp::Node
{
public:
  SerialCommunication() : Node("serial"){

    // Create a publisher to send messages to the microcontroller
    publisher_ = this->create_publisher<std_msgs::msg::String>("serial_send", 10); //(topic_name, queue_size)
    //timer_ = this->create_wall_timer(
    //1100ms, std::bind(&SerialCommunication::serial_send_callback, this));
    // Create a subscriber to receive messages from the microcontroller
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "serial_receive", 10, std::bind(&SerialCommunication::serial_receive_callback, this, std::placeholders::_1));
    
       // Open the serial port
    serial_port_ = open("/dev/serial/by-path/pci-0000:00:14.0-usb-0:5:1.0-port0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port_ == -1){
      std::cout << "Failed to open serial port" << std::endl;
      return;
    }
    else{
      RCLCPP_INFO(this->get_logger(), "Serial Port Opened :) ");
    }

    // Configure the serial port
    struct termios options;
    tcgetattr(serial_port_, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcsetattr(serial_port_, TCSANOW, &options);


    // Start the serial communication loop
    serial_send_loop();
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  int serial_port_;
  //rclcpp::TimerBase::SharedPtr timer_;


  void serial_send_loop(){
    while (rclcpp::ok()){
      // Read from the serial port
      char read_buffer[256];
      int bytes_read = read(serial_port_, &read_buffer, sizeof(read_buffer));
      if (bytes_read > 0)
      {
        // Publish the received message
        std_msgs::msg::String message;
        message.data = std::string(read_buffer, bytes_read);
        publisher_->publish(message);
      }

      // Spin the node to receive messages
      //rclcpp::spin_some(this);

      // Sleep for a short time to avoid using too much CPU
      std::this_thread::sleep_for(1000ms);
    }
  }

  void serial_receive_callback(const std_msgs::msg::String::SharedPtr message)
  {
    // Write to the serial port
    write(serial_port_, message->data.c_str(), message->data.length());
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialCommunication>());
  rclcpp::shutdown();
  return 0;
}
