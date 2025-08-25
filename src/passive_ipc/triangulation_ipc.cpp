#include "triangulation_ipc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

TriangulationNode::TriangulationNode(const sensor_msgs::msg::CameraInfo & camera_info)
: Node("triangulation_rgb", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    this->declare_parameter("frame_id", "left_camera_link");
    this->declare_parameter<int>("sampling_factor", 4);
    this->declare_parameter<double>("crop_factor", 1);

    frame_id_ = this->get_parameter("frame_id").as_string();
    sampling_factor_ = this->get_parameter("sampling_factor").as_int();

    fx_ = camera_info.p[0];
    fy_ = camera_info.p[5];
    principal_x_ = camera_info.p[2];
    principal_y_ = camera_info.p[6];

    RCLCPP_INFO(this->get_logger(), "fx: %f, fy: %f, cx: %f, cy: %f", fx_, fy_, principal_x_, principal_y_);

    // Subs diretos (sem message_filters, pois queremos IPC)
    sub_disp_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
        "disparity/image", 10,
        std::bind(&TriangulationNode::grab, this, std::placeholders::_1));

    sub_left_ = this->create_subscription<sensor_msgs::msg::Image>(
        "left/rect_image", 10,
        std::bind(&TriangulationNode::set_left, this, std::placeholders::_1));

    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
}

void TriangulationNode::set_left(std::unique_ptr<const sensor_msgs::msg::Image> msg)
{
    // guarda última imagem esquerda (cor) para usar no próximo disparity
    last_left_ = std::move(msg);
}

void TriangulationNode::grab(std::unique_ptr<const stereo_msgs::msg::DisparityImage> disp_msg)
{
    if (!last_left_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Sem imagem esquerda para colorir pointcloud");
        return;
    }

    // baseline e focal do disparity
    baseline_ = disp_msg->t;
    fx_ = disp_msg->f;

    int width  = disp_msg->image.width;
    int height = disp_msg->image.height;

    // preparar pointcloud
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header = disp_msg->header;
    cloud->header.frame_id = frame_id_;

    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);

    cloud->point_step = 16;

    std::vector<uint8_t> buffer;
    buffer.reserve((width/sampling_factor_)*(height/sampling_factor_)*cloud->point_step);

    // acesso direto aos dados do disparity
    const float *D = reinterpret_cast<const float*>(disp_msg->image.data.data());

    // conversão da imagem esquerda em OpenCV sem cópia
    cv_bridge::CvImageConstPtr cv_left;
    // Converte unique_ptr<const ImageMsg> -> shared_ptr<const ImageMsg>
    auto left_shared = std::shared_ptr<const ImageMsg>(last_left_.get(),
                                                    [](const ImageMsg*){});

    cv_left = cv_bridge::toCvShare(left_shared, left_shared->encoding);

    double crop_factor = this->get_parameter("crop_factor").as_double();
    crop_factor = std::clamp(crop_factor, 0.0, 1.0);

    int crop_width = static_cast<int>(width * crop_factor);
    int crop_height = static_cast<int>(height * crop_factor);
    int u0 = (width - crop_width) / 2;
    int v0 = (height - crop_height) / 2;
    int u1 = u0 + crop_width;
    int v1 = v0 + crop_height;

    for (int v = v0; v < v1; v += sampling_factor_) {
        for (int u = u0; u < u1; u += sampling_factor_) {
            float d = D[v*width + u];
            if (d > 1.0f) {
                float Z = -baseline_ * fx_ / d;
                float X = (u - principal_x_) * Z / fx_;
                float Y = (v - principal_y_) * Z / fy_;

                size_t off = buffer.size();
                buffer.resize(off + cloud->point_step);

                std::memcpy(&buffer[off + 0],  &X, sizeof(float));
                std::memcpy(&buffer[off + 4],  &Y, sizeof(float));
                std::memcpy(&buffer[off + 8],  &Z, sizeof(float));

                // cor BGR8
                cv::Vec3b bgr = cv_left->image.at<cv::Vec3b>(v, u);
                uint32_t rgb = (uint32_t(bgr[0]) << 16) | (uint32_t(bgr[1]) << 8) | (uint32_t(bgr[2]));
                float rgb_float;
                std::memcpy(&rgb_float, &rgb, sizeof(float));
                std::memcpy(&buffer[off + 12], &rgb_float, sizeof(float));
            }
        }
    }

    cloud->width = buffer.size() / cloud->point_step;
    cloud->height = 1;
    cloud->row_step = cloud->point_step * cloud->width;
    cloud->is_dense = false;
    cloud->data = std::move(buffer);

    pub_cloud_->publish(std::move(cloud));
}
