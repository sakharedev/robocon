#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp> // Add this line

using namespace std::chrono_literals;

class DistanceNode : public rclcpp::Node
{
    DistanceNode() : Node("DistanceNode")
    {
        
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std:: make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}