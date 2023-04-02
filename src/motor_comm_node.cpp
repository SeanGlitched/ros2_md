#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h"



#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>



class SerialNode : public rclcpp::Node
{
public:
    SerialNode(char* arg__)
        : Node("serial_node"), port_name_(arg__), baud_rate_(115200)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("serial_data", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "serial_send", 10, std::bind(&SerialNode::serial_send_callback, this, std::placeholders::_1));

        try
        {
            serial_.setPort(port_name_);
            serial_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port %s", port_name_.c_str());
        }

        if (serial_.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Successfully opened port %s", port_name_.c_str());
            serial_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                    std::bind(&SerialNode::serial_read_callback, this));
        }
    }

private:
    void serial_send_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (serial_.isOpen())
        {
            try
            {
                serial_.write(msg->data);
            }
            catch (const serial::IOException &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error writing to the serial port");
            }
        }
    }

    void serial_read_callback()
    {
        if (serial_.available())
        {
            try
            {
                std::string data = serial_.readline();
                auto message = std_msgs::msg::String();
                message.data = data;
                publisher_->publish(message);
            }
            catch (const serial::IOException &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error reading from the serial port");
            }
        }
    }

    std::string port_name_;
    int baud_rate_;
    serial::Serial serial_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr serial_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <port>" << std::endl;
        return 1;
    }

    rclcpp::spin(std::make_shared<SerialNode>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
