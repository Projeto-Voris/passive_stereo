#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tf2/LinearMath/Transform.h"


class TriangulationNode : public rclcpp::Node {
public:

  explicit TriangulationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callback da disparidade (único ponteiro para IPC)
  void grab(std::unique_ptr<const stereo_msgs::msg::DisparityImage> disp_msg);
  void grabcamInfoRight(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  // Callback da imagem esquerda retificada (colorida BGR8)
  void set_left(sensor_msgs::msg::Image::SharedPtr msg);

  // Subs
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr sub_disp_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;


  // Pub
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;

  // Última imagem esquerda guardada (shared to preserve IPC semantics)
  sensor_msgs::msg::Image::SharedPtr last_left_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Intrínsecos
  bool receive_camera_info_ {false};
  bool tf_static_cached_{false};
  float fx_{0.0f}, fy_{0.0f};
  float principal_x_{0.0f}, principal_y_{0.0f};
  float baseline_{0.0f};

  std::string frame_id_{"left_camera_link"};
  std::string base_frame_{"base_link"};
  tf2::Transform T_base_cam_;
  tf2::Matrix3x3 tf_cam2ros ={0.0, 0.0, 1.0,   // row 0
                              -1.0, 0.0, 0.0,   // row 1
                                0.0,-1.0, 0.0};  // row 2

  // Parâmetro de amostragem
  float sampling_factor_{0.5f};
};
