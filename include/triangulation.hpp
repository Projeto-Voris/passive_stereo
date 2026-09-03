#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"

#include "triangulation_cuda.cuh"

class TriangulationNode : public rclcpp::Node {
public:
  explicit TriangulationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TriangulationNode() override = default;

private:
  // Callbacks
  void grab(std::unique_ptr<const stereo_msgs::msg::DisparityImage> disp_msg);
  void grabcamInfoRight(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void set_left(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void update_transform_matrix();

  size_t triangulate_cpu(
      const float* disp_data,
      int disp_step,
      const uint8_t* img_data,
      int img_step,
      const passive_stereo::TriangulationParams& params,
      passive_stereo::PointXYZRGB* out_points,
      size_t max_points);

  // Subscriptions & Publishers
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr sub_disp_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;

  // Thread-safe storage for last left image
  std::mutex left_img_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr last_left_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Intrinsics
  bool receive_camera_info_{false};
  bool tf_static_cached_{false};
  bool has_parent_{false};
  float fx_{0.0f}, fy_{0.0f};
  float principal_x_{0.0f}, principal_y_{0.0f};
  float baseline_{0.0f};

  // Combined affine transformation matrix & translation
  float R_combined_[9]{1.0f, 0.0f, 0.0f,
                       0.0f, 1.0f, 0.0f,
                       0.0f, 0.0f, 1.0f};
  float T_combined_[3]{0.0f, 0.0f, 0.0f};

  std::string frame_id_{"left_camera_link"};
  std::string parent_frame_{""};
  tf2::Transform T_base_cam_;
  tf2::Matrix3x3 tf_cam2ros_{0.0, 0.0, 1.0,
                            -1.0, 0.0, 0.0,
                             0.0,-1.0, 0.0};

  // Parameters
  bool use_gpu_{true};
  float sampling_factor_{0.5f};
  double crop_factor_{1.0};
  double max_dist_{10.0};
  double min_disp_{1.0};

  cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE();
  bool apply_clahe_ {false};

  // GPU Triangulator (owns its own pinned in/out buffers — no host_point_buffer_ needed)
  std::unique_ptr<passive_stereo::CudaTriangulator> cuda_triangulator_;

  // Reusable buffer for the CPU-only fallback path
  std::vector<passive_stereo::PointXYZRGB> cpu_point_buffer_;
};
