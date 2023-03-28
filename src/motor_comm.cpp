#include <cstdio>
#include <iostream>
#include <string>

//Allows use of most common systems is ROS2
#include "rclcpp/rclcpp.hpp"
//Allows you the build in message type to publish data
#include "std_msgs/msg/string.hpp"

//#include "ros2_md/msg/motor_command.hpp"

#include "ros2_md/srv/read_encoder.hpp"
//serial related
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


using namespace std::chrono_literals;

#define DEBUG 0

//inherit from the Node::cpp class
/*
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}
*/





class MotorCommunication : public rclcpp::Node
{
public:

    // get_encoder  (service)
    // write_command (topic)

    MotorCommunication(char* arg) : Node("motor_driver"), arg__(arg){

        // Create a publisher to send messages to the microcontroller
        //publisher_ = this->create_publisher<std_msgs::msg::String>("serial_send", 10); //(topic_name, queue_size)

        
        // Create a subscriber to receive messages from the microcontroller
        write_command_ = this->create_subscription<std_msgs::msg::String>(
            "write_command", 10, std::bind(&MotorCommunication::write_command_callback, this, std::placeholders::_1));

        // Service
        service_ = this->create_service<ros2_md::srv::ReadEncoder>(
            "read_encoder", std::bind(&MotorCommunication::handle_service, this, std::placeholders::_1, std::placeholders::_2));

        // Open the serial port
        serial_port_ = open(arg, O_RDWR | O_NOCTTY | O_NDELAY);
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
        //serial_send_loop();
    }
    

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr write_command_;
    // Service
    rclcpp::Service<ros2_md::srv::ReadEncoder>::SharedPtr service_;
    int serial_port_;
    char* arg__;

    void write_command_callback(const std_msgs::msg::String::SharedPtr message)
    {
        // Write to the serial port
        write(serial_port_, message->data.c_str(), message->data.length());
    }

      // Handler function for service
    void handle_service(
        //const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ros2_md::srv::ReadEncoder::Request> request,
        std::shared_ptr<ros2_md::srv::ReadEncoder::Response> response)
    {
            response->encoder = request->command;
            
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <message>" << std::endl;
        return 1;
    }
    rclcpp::spin(std::make_shared<MotorCommunication>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
