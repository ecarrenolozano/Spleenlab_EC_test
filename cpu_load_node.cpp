#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CPULoadPublisher : public rclcpp::Node
{
public:
    CPULoadPublisher() : Node("cpu_load_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("cpu_load", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(5),
                                         std::bind(&CPULoadPublisher::publish_load, this));
    }

private:
    void publish_load()
    {
        std::ifstream cpu_info("/proc/loadavg");
        std::string load;
        if (cpu_info.is_open())
        {
            std::getline(cpu_info, load);
            cpu_info.close();
            auto message = std_msgs::msg::String();
            message.data = load;
            RCLCPP_INFO(this->get_logger(), "Publishing CPU load: %s", message.data.c_str());
            publisher_->publish(message);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open /proc/loadavg");
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CPULoadPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}