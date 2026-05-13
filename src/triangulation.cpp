#include "triangulation.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

TriangulationNode::TriangulationNode(const rclcpp::NodeOptions & options)
: Node("triangulation_rgb", options)
{
    this->declare_parameter("frame_id", "left_camera_link");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("sampling_factor", 0.5);
    this->declare_parameter("crop_factor", 1.0);
    this->declare_parameter("max_dist", 10.0);

    frame_id_ = this->get_parameter("frame_id").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    sampling_factor_ = this->get_parameter("sampling_factor").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos_pub_profile = rclcpp::SensorDataQoS();
    qos_pub_profile.keep_last(1);

    auto disp_cb_group = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

    // 2. Configure as opções de inscrição
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = disp_cb_group;
    
    // Subs diretos (sem message_filters, pois queremos IPC)
    sub_disp_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
        "disparity/image", rclcpp::SensorDataQoS(), 
        std::bind(&TriangulationNode::grab, this, std::placeholders::_1), sub_options);

    right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "right/camera_info", rclcpp::SensorDataQoS(),
        std::bind(&TriangulationNode::grabcamInfoRight, this, std::placeholders::_1), sub_options);
    sub_left_ = this->create_subscription<sensor_msgs::msg::Image>(
        "left/image_rect", rclcpp::SensorDataQoS(),
        std::bind(&TriangulationNode::set_left, this, std::placeholders::_1), sub_options);

    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("disparity/pointcloud", qos_pub_profile);
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

    if (!tf_static_cached_){
    try {
            auto tf_base_cam = tf_buffer_->lookupTransform(
                base_frame_, frame_id_, tf2::TimePointZero);
            tf2::fromMsg(tf_base_cam.transform, T_base_cam_);
            tf_static_cached_ = true;
            RCLCPP_INFO(this->get_logger(), "Static transform [%s -> %s] successfully cached!", base_frame_.c_str(), frame_id_.c_str());
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Wait static TF to be available: %s", ex.what());
            return; // Retorna cedo pois não podemos publicar path/poses corretos sem essa TF
        }
    }

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
    cloud->header.frame_id = base_frame_;

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

    int num_channels = cv_left->image.channels();

    for (int v = v0; v < v1; v += step) {
        // Lemos a linha como bytes puros (uchar) em vez de forçar Vec3b
        const uchar* row = cv_left->image.ptr<uchar>(v); 
        
        for (int u = u0; u < u1; u += step) {
            float d = D[v*width + u];
            if (d > 1.0f) {
                float Z = -baseline_ * fx_ / d;
                float X = (u - principal_x_) * Z / fx_;
                float Y = (v - principal_y_) * Z / fy_;

                tf2::Vector3 pt_disp(X, Y, Z);
                tf2::Vector3 pt_disp_ros = tf_cam2ros * pt_disp;
                tf2::Vector3 pt_base = pt_disp_ros + T_base_cam_.getOrigin();

                // float dist_sq = pt_base.length2();
                // float max_dist = this->get_parameter("max_dist").as_double();
                // float max_dist_sq = (max_dist * max_dist);
                // if (dist_sq < max_dist_sq){

                    float final_x = pt_base.x();
                    float final_y = pt_base.y();
                    float final_z = pt_base.z();

                    std::memcpy(ptr + idx, &final_x, sizeof(float));
                    std::memcpy(ptr + idx + 4, &final_y, sizeof(float));
                    std::memcpy(ptr + idx + 8, &final_z, sizeof(float));

                    uint32_t rgb = 0;
                    
                    // Tratamento correto de cores com base no número de canais
                    if (num_channels == 1) {
                        // Escala de cinza: 1 byte por pixel
                        uint8_t intensity = row[u];
                        rgb = (uint32_t(intensity) << 16) | (uint32_t(intensity) << 8) | (uint32_t(intensity));
                    } else if (num_channels == 3) {
                        // Colorida (assumindo BGR): 3 bytes por pixel
                        uint8_t b = row[u * 3 + 0];
                        uint8_t g = row[u * 3 + 1];
                        uint8_t r = row[u * 3 + 2];
                        rgb = (uint32_t(b) << 16) | (uint32_t(g) << 8) | (uint32_t(r));
                    }

                    std::memcpy(ptr + idx + 12, &rgb, sizeof(rgb));

                    idx += cloud->point_step;
                }
            // }
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