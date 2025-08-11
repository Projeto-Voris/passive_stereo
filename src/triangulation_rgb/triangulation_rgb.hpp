#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <std_msgs/msg/int16_multi_array.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <stereo_msgs/msg/disparity_image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>

class TriangulationNode : public rclcpp::Node
{
    public:
        TriangulationNode(sensor_msgs::msg::CameraInfo camera_info);

    private:
        using ImageMsg = stereo_msgs::msg::DisparityImage;
        typedef message_filters::sync_policies::ApproximateTime<stereo_msgs::msg::DisparityImage, sensor_msgs::msg::Image> approximate_sync_policy;

        void GrabImages(const ImageMsg::ConstSharedPtr disp_msg, const sensor_msgs::msg::Image::ConstSharedPtr left_msg);

        float baseline_, principal_x_, principal_y_, fx_, fy_, f_;
        int sampling_factor;


        cv_bridge::CvImagePtr cv_ptr_disp;
        cv_bridge::CvImageConstPtr cv_ptr_left;

        sensor_msgs::msg::CameraInfo camera_info_;

        std::shared_ptr<message_filters::Subscriber<stereo_msgs::msg::DisparityImage>> disparity_sub;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub;
        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
};