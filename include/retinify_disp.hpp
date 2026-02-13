#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <retinify/retinify.hpp>

class RetinifyDisparityNode : public rclcpp::Node {
public:
    using approximate_sync_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    explicit RetinifyDisparityNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void grabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr msgRight);
    void grabcamInfoLeft(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void grabcamInfoRight(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    bool retinifyCalibParam();
    bool CalculateRectificationRemaps();
    void RectifyImages(const cv::Mat& imgL, const cv::Mat& imgR, const sensor_msgs::msg::Image::ConstSharedPtr msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr msgRight);
    void publishColoredPointCloud(const std::vector<float> & points, const cv::Mat & color_img, const std_msgs::msg::Header & header);
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
    
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> sync_;

    // Processing state
    bool publish_rectified_ {false};
    bool publish_disp {false};
    bool debug_image_ {false};
    bool left_info_received_ {false};
    bool right_info_received_ {false};
    bool maps_initialized_ {false};

    // Camera Parameters & Maps
    sensor_msgs::msg::CameraInfo left_camera_info_;
    sensor_msgs::msg::CameraInfo right_camera_info_;
    cv::Mat left_map1, left_map2, right_map1, right_map2;
    cv::Mat rectImgL, rectImgR;
    double focal_length_ {0.0};
    double baseline_ {0.0};

    retinify::Pipeline pipeline;
    retinify::CalibrationParameters calib_;

    // Publishers
    rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr pub_disp_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_left_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_right_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_disp_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
};