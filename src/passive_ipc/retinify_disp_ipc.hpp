#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <retinify/retinify.hpp>

class RetinifyDisparityNode : public rclcpp::Node {
    public:
        using ImageMsg = sensor_msgs::msg::Image;
        using DisparityMsg = stereo_msgs::msg::DisparityImage;
        using approximate_sync_policy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

        explicit RetinifyDisparityNode(sensor_msgs::msg::CameraInfo infoL, sensor_msgs::msg::CameraInfo infoR);

    private:
        void grabStereo(const ImageMsg::ConstSharedPtr msgLeft,
                        const ImageMsg::ConstSharedPtr msgRight);
        void RectifyImages(cv::Mat imgL, cv::Mat imgR, const sensor_msgs::msg::Image::ConstSharedPtr msgLeft);
        void CalculateRectificationRemaps();

        // Subs
        std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub_;
        std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub_;
        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> sync_;

        cv::Mat left_map1, left_map2;
        cv::Mat right_map1, right_map2;

        cv::Mat rectImgL, rectImgR;

        cv_bridge::CvImageConstPtr cv_ptrLeft;
        cv_bridge::CvImageConstPtr cv_ptrRight;

        bool publish_rectified, debug_image;

        retinify::tools::StereoMatchingPipeline pipeline;

        sensor_msgs::msg::CameraInfo left_camera_info;
        sensor_msgs::msg::CameraInfo right_camera_info;

        // Pub
        rclcpp::Publisher<DisparityMsg>::SharedPtr pub_disp_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_left_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_right_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_disp_publisher;

        // Calibração
        double focal_length_ {0.0};
        double baseline_ {0.0};
};
