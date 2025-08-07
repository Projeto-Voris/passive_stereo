#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <std_msgs/msg/int16_multi_array.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <stereo_msgs/msg/disparity_image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>

#include <retinify/retinify.hpp>





class RetinityDisparityNode : public rclcpp::Node
{
    public:
        RetinityDisparityNode(sensor_msgs::msg::CameraInfo infoL, sensor_msgs::msg::CameraInfo infoR);

    private:
        using ImageMsg = sensor_msgs::msg::Image;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

        void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr msgRight);
        void RectifyImages(cv::Mat imgL, cv::Mat imgR);
        void CalculateRectificationRemaps();

        cv::Mat left_map1, left_map2;
        cv::Mat right_map1, right_map2;

        cv::Mat rectImgL, rectImgR;

        cv_bridge::CvImageConstPtr cv_ptrLeft;
        cv_bridge::CvImageConstPtr cv_ptrRight;

        float focal_length, baseline;
        bool publish_rectified, resize_input, resize_disparity;

        retinify::tools::StereoMatchingPipeline pipeline;

        sensor_msgs::msg::CameraInfo left_camera_info;
        sensor_msgs::msg::CameraInfo right_camera_info;

        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub;

        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

        rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_left_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_right_publisher;
};