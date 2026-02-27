#include "triangulation.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

TriangulationNode::TriangulationNode(const rclcpp::NodeOptions & options)
: Node("triangulation_rgb", options)
{
    this->declare_parameter("frame_id", "left_camera_link");
    this->declare_parameter("sampling_factor", 0.5);
    this->declare_parameter("crop_factor", 1.0);

    frame_id_ = this->get_parameter("frame_id").as_string();
    sampling_factor_ = this->get_parameter("sampling_factor").as_double();

    RCLCPP_INFO(this->get_logger(), "fx: %f, fy: %f, cx: %f, cy: %f", fx_, fy_, principal_x_, principal_y_);

    // Subs diretos (sem message_filters, pois queremos IPC)
    sub_disp_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
        "disparity/image", 10,
        std::bind(&TriangulationNode::grab, this, std::placeholders::_1));

    right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "right/camera_info", 10,
        std::bind(&TriangulationNode::grabcamInfoRight, this, std::placeholders::_1));
    sub_left_ = this->create_subscription<sensor_msgs::msg::Image>(
        "left/image_rect", 10,
        std::bind(&TriangulationNode::set_left, this, std::placeholders::_1));

    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("disparity/pointcloud", 10);
}

void TriangulationNode::grabcamInfoRight(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    
    // Receive and save intrinsic parameters from projection matrix
    if(receive_camera_info_) return; // Only receive once
    fx_ = msg->p[0];
    fy_ = msg->p[5];
    principal_x_ = msg->p[2];
    principal_y_ = msg->p[6];
    receive_camera_info_ = true;
    RCLCPP_INFO(this->get_logger(), "Received camera info. fx: %f, fy: %f, cx: %f, cy: %f", fx_, fy_, principal_x_, principal_y_);

}
void TriangulationNode::set_left(sensor_msgs::msg::Image::SharedPtr msg)
{
    last_left_ = std::move(msg);
}
void TriangulationNode::grab(std::unique_ptr<const stereo_msgs::msg::DisparityImage> disp_msg)
{
    if (!last_left_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Sem imagem esquerda para colorir pointcloud");
        return;
    }
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Disp stamp: %u.%u", disp_msg->header.stamp.sec, disp_msg->header.stamp.nanosec);

    // fetch dynamic parameters once per message
    sampling_factor_ = this->get_parameter("sampling_factor").as_double();
    double crop_factor = this->get_parameter("crop_factor").as_double();
    sampling_factor_ = std::clamp(sampling_factor_, 0.0f, 1.0f);
    crop_factor = std::clamp(crop_factor, 0.0, 1.0);

    // baseline and focal from the disparity message
    baseline_ = disp_msg->t;
    fx_ = disp_msg->f;
    int width  = disp_msg->image.width;
    int height = disp_msg->image.height;

    // prepare pointcloud (reuse buffer later if desired)
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header = disp_msg->header;
    cloud->header.stamp = this->get_clock()->now();
    cloud->header.frame_id = frame_id_;

    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    cloud->point_step = 16;

    int step = sampling_factor_ > 0.0 ? std::max(1, static_cast<int>(1.0 / sampling_factor_)) : 1;
    int crop_width = static_cast<int>(width * crop_factor);
    int crop_height = static_cast<int>(height * crop_factor);
    int u0 = (width - crop_width) / 2;
    int v0 = (height - crop_height) / 2;
    int u1 = u0 + crop_width;
    int v1 = v0 + crop_height;

    const float *D = reinterpret_cast<const float*>(disp_msg->image.data.data());
    cv_bridge::CvImageConstPtr cv_left = cv_bridge::toCvShare(last_left_, last_left_->encoding);

    // allocate maximum possible size once and write directly
    size_t max_pts = ((crop_width + step - 1) / step) * ((crop_height + step - 1) / step);
    cloud->data.resize(max_pts * cloud->point_step);
    uint8_t *ptr = cloud->data.data();
    size_t idx = 0;

    for (int v = v0; v < v1; v += step) {
        const cv::Vec3b* row = cv_left->image.ptr<cv::Vec3b>(v);
        for (int u = u0; u < u1; u += step) {
            float d = D[v*width + u];
            if (d > 1.0f) {
                float Z = -baseline_ * fx_ / d;
                float X = (u - principal_x_) * Z / fx_;
                float Y = (v - principal_y_) * Z / fy_;

                std::memcpy(ptr + idx, &X, sizeof(float));
                std::memcpy(ptr + idx + 4, &Y, sizeof(float));
                std::memcpy(ptr + idx + 8, &Z, sizeof(float));

                const cv::Vec3b &bgr = row[u];
                uint32_t rgb = (uint32_t(bgr[2]) << 16) | (uint32_t(bgr[1]) << 8) | (uint32_t(bgr[0]));
                std::memcpy(ptr + idx + 12, &rgb, sizeof(rgb));

                idx += cloud->point_step;
            }
        }
    }

    // shrink to actual size and publish
    cloud->width = idx / cloud->point_step;
    cloud->height = 1;
    cloud->row_step = idx;
    cloud->is_dense = false;
    cloud->data.resize(idx);

    pub_cloud_->publish(std::move(cloud));
}
RCLCPP_COMPONENTS_REGISTER_NODE(TriangulationNode)