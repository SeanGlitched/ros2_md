#include <cstdio>
#include <iostream>
#include <string>

//Allows use of most common systems is ROS2
#include "rclcpp/rclcpp.hpp"
//Allows you the build in message type to publish data
#include "std_msgs/msg/string.hpp"

//#include "motor_interfaces/msg/motor_command.hpp"

#include "motor_interfaces/srv/read_encoder.hpp"
//serial related
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;

#define DEBUG 0

class MotorCommunication : public rclcpp::Node
{
public:

    // get_encoder  (service)
    // write_command (topic)

    MotorCommunication(char* arg) : Node("motor_driver"){

        // Create a publisher to send messages to the microcontroller
        publisher_ = this->create_publisher<std_msgs::msg::String>("serial_send", 10); //(topic_name, queue_size)

        
        // Create a subscriber to receive messages from the microcontroller
        write_command_ = this->create_subscription<std_msgs::msg::String>(
            "write_command", 10, std::bind(&MotorCommunication::write_command_callback, this, std::placeholders::_1));

        // Service
        service_ = this->create_service<motor_interfaces::srv::ReadEncoder>(
            "read_encoder", std::bind(&MotorCommunication::handle_service, this, std::placeholders::_1, std::placeholders::_2));

        // Open the serial port
            
        int spp = open(arg, O_RDWR | O_NOCTTY | O_NDELAY);
        if (spp == -1){
            std::cout << "Failed to open serial port" << std::endl;
            return;
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Serial Port Opened :) ");
        }

        // Configure the serial port
        struct termios options;
        tcgetattr(spp, &options);
        options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        options.c_iflag = 0;
        options.c_oflag = 0;
        options.c_lflag = 0;
        tcsetattr(spp, TCSANOW, &options);
        
        sp_ = spp;
        


        // Start the serial communication loop
        serial_send_loop();
    }
    

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr write_command_;
    // Service
    rclcpp::Service<motor_interfaces::srv::ReadEncoder>::SharedPtr service_;
    int sp_;
    char* arg__;

    void serial_send_loop(){
        while (rclcpp::ok()){

            char read_buffer[1024];
            int bytes_read = read(sp_,&read_buffer,sizeof(read_buffer));
            if (bytes_read > 0){
                // Publish the received message
                std_msgs::msg::String message;
                message.data = std::string(read_buffer, bytes_read);
                publisher_->publish(message);
            }
        // Sleep for a short time to avoid using too much CPU
            std::this_thread::sleep_for(1000ms);
        }
    }
    void write_command_callback(const std_msgs::msg::String::SharedPtr message)
    {
        // Write to the serial port
        write(sp_,message->data.c_str(), message->data.length());
    }

    // Handler function for service
    void handle_service(
        //const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<motor_interfaces::srv::ReadEncoder::Request> request,
        std::shared_ptr<motor_interfaces::srv::ReadEncoder::Response> response)
    {
            response->encoder_data = request->command;
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
