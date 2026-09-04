#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"

#include <retinify/retinify.hpp>
#include "triangulation_cuda.cuh"

namespace passive_stereo
{

class RetinifyStereoNode : public rclcpp::Node
{
public:
    using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using ExactSyncPolicy = message_filters::sync_policies::ExactTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    explicit RetinifyStereoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~RetinifyStereoNode() override;

private:
    // Callbacks
    void onCameraInfoLeft(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void onCameraInfoRight(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void onStereoImages(
        const sensor_msgs::msg::Image::ConstSharedPtr msg_left,
        const sensor_msgs::msg::Image::ConstSharedPtr msg_right);

    // Helper methods
    bool initializePipeline(uint32_t width, uint32_t height);
    void updateTransformMatrix();
    cv::Mat applyCLAHE(const cv::Mat & input_bgr);

    size_t triangulateCPU(
        const float* disp_data,
        int disp_step,
        const uint8_t* img_data,
        int img_step,
        const TriangulationParams& params,
        size_t max_points,
        void* out_points,
        bool with_confidence);

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_left_info_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_right_info_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_left_img_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_right_img_;

    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> approx_sync_;
    std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> exact_sync_;

    // Publishers
    rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr pub_disparity_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rect_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rect_right_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_debug_disp_;

    // Retinify pipeline
    retinify::Pipeline pipeline_;
    bool pipeline_initialized_{false};
    std::mutex pipeline_mutex_;

    // Pinned Host Disparity Buffer
    float* h_pinned_disp_{nullptr};
    size_t pinned_disp_bytes_{0};
    std::vector<float> cpu_disp_buffer_;

    // GPU CUDA Triangulator
    std::unique_ptr<passive_stereo::CudaTriangulator> cuda_triangulator_;
    std::vector<uint8_t> cpu_point_buffer_;

    // Optional Retinify Buffers
    std::vector<float> depth_buffer_;
    std::vector<uint8_t> rect_left_buffer_;
    std::vector<uint8_t> rect_right_buffer_;

    // Camera Calibration State
    std::mutex calib_mutex_;
    sensor_msgs::msg::CameraInfo left_camera_info_;
    sensor_msgs::msg::CameraInfo right_camera_info_;
    bool left_info_received_{false};
    bool right_info_received_{false};
    double fx_{0.0};
    double fy_{0.0};
    double cx_{0.0};
    double cy_{0.0};
    double baseline_{0.0};

    // TF Transforms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2::Transform T_base_cam_;
    bool tf_static_cached_{false};
    float R_combined_[9]{1.0f, 0.0f, 0.0f,
                         0.0f, 1.0f, 0.0f,
                         0.0f, 0.0f, 1.0f};
    float T_combined_[3]{0.0f, 0.0f, 0.0f};

    // CLAHE
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE();

    // Node Parameters
    bool publish_disparity_{true};
    bool publish_pointcloud_{true};
    bool publish_depth_{false};
    bool publish_rectified_{false};
    bool debug_image_{false};
    bool use_exact_sync_{false};
    bool apply_clahe_{false};
    bool use_gpu_{true};
    bool publish_confidence_field_{true};
    int confidence_radius_{2};
    double confidence_alpha_{2.0};
    double min_confidence_{0.35};
    bool invert_x_{false};
    bool invert_y_{false};
    bool invert_z_{false};
    std::string depth_mode_str_{"accurate"};
    std::string calibration_file_{""};
    std::string frame_id_{"left_camera_link"};
    std::string parent_frame_{""};
    double sampling_factor_{1.0};
    double crop_factor_{1.0};
    double min_disp_{1.0};
    double max_dist_{15.0};
    int sync_queue_size_{10};
};

} // namespace passive_stereo
