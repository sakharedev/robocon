#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
    public:
        Publisher() : Node("publisher_node"), count(0)
        {
            publisher = this->create_publisher<std_msgs::msg::String>("chatter", 1);
            timer = this->create_wall_timer(100ms, std::bind(&Publisher::timer_callback, this));
        }
    
    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hi i am Paras " + std::to_string(count++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        size_t count;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}