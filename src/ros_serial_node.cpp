#include <rclcpp/rclcpp.hpp>
#include <rclcpp_serial/serial_port.hpp>
#include <std_msgs/msg/string.hpp>

class SerialPortNode : public rclcpp::Node
{
public:
  SerialPortNode()
  : Node("serial_port_node")
  {
    // Create a SerialPort object
    std::string port_name = "/dev/ttyUSB0";
    int baud_rate = 115200;
    serial_port_ = std::make_unique<rclcpp_serial::SerialPort>(port_name, baud_rate);

    // Open the serial port
    if (!serial_port_->open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port");
      return;
    }

    // Create a subscriber to receive messages from ROS
    subscriber_ = create_subscription<std_msgs::msg::String>(
      "serial_tx", 10, std::bind(&SerialPortNode::onSerialTx, this, std::placeholders::_1));

    // Create a publisher to send messages to ROS
    publisher_ = create_publisher<std_msgs::msg::String>("serial_rx", 10);

    // Create a timer to read data from the serial port
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&SerialPortNode::onTimer, this));
  }

private:
  void onSerialTx(const std_msgs::msg::String::SharedPtr msg)
  {
    // Send the message to the serial port
    std::vector<uint8_t> buffer(msg->data.begin(), msg->data.end());
    serial_port_->write(buffer);
  }

  void onTimer()
  {
    // Read data from the serial port
    std::vector<uint8_t> buffer;
    size_t num_bytes = serial_port_->read(buffer);

    // Process the received data
    if (num_bytes > 0) {
      // Convert the buffer to a string
      std::string message(buffer.begin(), buffer.end());

      // Print the received message
      RCLCPP_INFO(get_logger(), "Received message: %s", message.c_str());

      // Publish the message to ROS
      auto msg = std_msgs::msg::String();
      msg.data = message;
      publisher_->publish(msg);
    }
  }

  std::unique_ptr<rclcpp_serial::SerialPort> serial_port_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SerialPortNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
