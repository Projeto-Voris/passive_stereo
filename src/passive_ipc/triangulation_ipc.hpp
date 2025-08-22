#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class TriangulationNode : public rclcpp::Node {
public:
  using ImageMsg = sensor_msgs::msg::Image;
  using DisparityMsg = stereo_msgs::msg::DisparityImage;

  explicit TriangulationNode(const sensor_msgs::msg::CameraInfo & camera_info);

private:
  // Callback da disparidade (único ponteiro para IPC)
  void grab(std::unique_ptr<const DisparityMsg> disp_msg);

  // Callback da imagem esquerda retificada (colorida BGR8)
  void set_left(std::unique_ptr<const ImageMsg> msg);

  // Subs
  rclcpp::Subscription<DisparityMsg>::SharedPtr sub_disp_;
  rclcpp::Subscription<ImageMsg>::SharedPtr sub_left_;

  // Pub
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;

  // Última imagem esquerda guardada
  std::unique_ptr<const ImageMsg> last_left_;

  // Intrínsecos
  float fx_{0.0f}, fy_{0.0f};
  float principal_x_{0.0f}, principal_y_{0.0f};
  float baseline_{0.0f};
  std::string frame_id_{"left_camera_link"};

  // Parâmetro de amostragem
  int sampling_factor_{4};
};
