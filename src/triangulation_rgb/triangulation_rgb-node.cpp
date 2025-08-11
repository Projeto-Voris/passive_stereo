#include "triangulation_rgb.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string info_topic = (argc > 1) ? argv[1] : "/left/camera_info";

    auto camera_info = sensor_msgs::msg::CameraInfo();

    bool camera_info_received = false;

    auto info_node = std::make_shared<rclcpp::Node>("camera_info_subscriber");

    camera_info_received = rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(camera_info, info_node,info_topic, std::chrono::seconds(5));

    if(camera_info_received){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera info received");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera info topic: %s", info_topic.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No camera info provided");
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Camera info topic: %s", info_topic.c_str());
    }
    auto node = std::make_shared<TriangulationNode>(camera_info);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}