#include "disparity.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: disparity_node <left_info_topic> <right_info_topic>");
        return 1;
    }

    std::string left_info_topic = argv[1];
    std::string right_info_topic = argv[2];

    auto right_camera_info = sensor_msgs::msg::CameraInfo();
    auto left_camera_info = sensor_msgs::msg::CameraInfo();

    bool left_camera_info_received = false;
    bool right_camera_info_received = false;

    auto info_node = std::make_shared<rclcpp::Node>("disparity_camera_info_subscriber");

    right_camera_info_received = rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(right_camera_info, info_node,right_info_topic, std::chrono::seconds(5));
    left_camera_info_received = rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(left_camera_info, info_node,left_info_topic, std::chrono::seconds(5));

    if(left_camera_info_received && right_camera_info_received){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera info received");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left info topic: %s", left_info_topic.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right info topic: %s", right_info_topic.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right Projection Matrix P[3] value: %f", right_camera_info.p[3]);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No camera info provided");
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Left info topic: %s", left_info_topic.c_str());
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Right info topic: %s", right_info_topic.c_str());
    }

    auto node = std::make_shared<PassiveStereoNode>(left_camera_info, right_camera_info);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}