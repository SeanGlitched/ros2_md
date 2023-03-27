#include <iostream>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <message>" << std::endl;
        return 1;
    }

    auto message = std::string(argv[1]);

    std::cout << "Received message: " << message << std::endl;

    rclcpp::shutdown();
    return 0;
}