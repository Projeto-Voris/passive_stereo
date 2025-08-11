#include "triangulation.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string info_topic = "/left/camera_info";

    auto camera_info = sensor_msgs::msg::CameraInfo();

    bool camera_info_received = false;

    auto info_node = std::make_shared<rclcpp::Node>("camera_info_subscriber");

    camera_info_received = rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(camera_info, info_node,info_topic, std::chrono::seconds(5));

    if(camera_info_received){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera info received");
    }
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No camera info provided");

    auto node = std::make_shared<TriangulationNode>(camera_info);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}