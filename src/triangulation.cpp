#include "triangulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <sensor_msgs/point_cloud2_iterator.hpp>

TriangulationNode::TriangulationNode(const rclcpp::NodeOptions & options)
: Node("triangulation_rgb", options)
{
    this->declare_parameter("frame_id", "left_camera_link");
    this->declare_parameter("parent_frame", "");
    this->declare_parameter("sampling_factor", 0.5);
    this->declare_parameter("crop_factor", 1.0);
    this->declare_parameter("max_dist", 10.0);
    this->declare_parameter("min_disp", 1.0);
    this->declare_parameter("use_gpu", true);
    this->declare_parameter("confidence_radius", 2);
    this->declare_parameter("confidence_alpha", 2.0);
    this->declare_parameter("min_confidence", 0.3);
    this->declare_parameter("clahe", false);
    
    frame_id_ = this->get_parameter("frame_id").as_string();
    parent_frame_ = this->get_parameter("parent_frame").as_string();
    sampling_factor_ = static_cast<float>(this->get_parameter("sampling_factor").as_double());
    crop_factor_ = this->get_parameter("crop_factor").as_double();
    max_dist_ = this->get_parameter("max_dist").as_double();
    min_disp_ = this->get_parameter("min_disp").as_double();
    use_gpu_ = this->get_parameter("use_gpu").as_bool();
    apply_clahe_ = this->get_parameter("clahe").as_bool();

    if (apply_clahe_){
        RCLCPP_INFO(this->get_logger(), "Applying CLAHE to Lab colorspace");
        clahe_->setClipLimit(5.0);
        clahe_->setTilesGridSize(cv::Size(5, 5));
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos_pub_profile = rclcpp::SensorDataQoS();
    qos_pub_profile.keep_last(1);

    auto disp_cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = disp_cb_group;

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

    // Initialize CUDA Triangulator
    cuda_triangulator_ = std::make_unique<passive_stereo::CudaTriangulator>();
    if (use_gpu_ && cuda_triangulator_->is_available()) {
        RCLCPP_INFO(this->get_logger(),
            "Triangulation initialized with CUDA GPU acceleration on: %s",
            cuda_triangulator_->get_device_name().c_str());
    } else {
        RCLCPP_INFO(this->get_logger(),
            "Triangulation initialized in OpenMP multi-threaded CPU mode (Max threads: %d)",
            omp_get_max_threads());
    }

    update_transform_matrix();
}

void TriangulationNode::update_transform_matrix()
{
    if (has_parent_ && tf_static_cached_) {
        tf2::Matrix3x3 R_mat = T_base_cam_.getBasis();
        tf2::Vector3 T_vec = T_base_cam_.getOrigin();

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R_combined_[r * 3 + c] = static_cast<float>(R_mat[r][c]);
            }
        }
        T_combined_[0] = static_cast<float>(T_vec.x());
        T_combined_[1] = static_cast<float>(T_vec.y());
        T_combined_[2] = static_cast<float>(T_vec.z());
    } else {
        R_combined_[0] = 1.0f; R_combined_[1] = 0.0f; R_combined_[2] = 0.0f;
        R_combined_[3] = 0.0f; R_combined_[4] = 1.0f; R_combined_[5] = 0.0f;
        R_combined_[6] = 0.0f; R_combined_[7] = 0.0f; R_combined_[8] = 1.0f;
        T_combined_[0] = 0.0f; T_combined_[1] = 0.0f; T_combined_[2] = 0.0f;
    }
}

void TriangulationNode::grabcamInfoRight(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
    if (receive_camera_info_) return;
    fx_ = static_cast<float>(msg->p[0]);
    fy_ = static_cast<float>(msg->p[5]);
    principal_x_ = static_cast<float>(msg->p[2]);
    principal_y_ = static_cast<float>(msg->p[6]);
    receive_camera_info_ = true;
    RCLCPP_INFO(this->get_logger(),
        "Received camera info. fx: %f, fy: %f, cx: %f, cy: %f",
        fx_, fy_, principal_x_, principal_y_);
}

void TriangulationNode::set_left(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(left_img_mutex_);
        last_left_ = msg;
    }

    parent_frame_ = this->get_parameter("parent_frame").as_string();
    has_parent_ = !parent_frame_.empty();

    if (has_parent_ && !tf_static_cached_) {
        try {
            auto tf_base_cam = tf_buffer_->lookupTransform(
                parent_frame_, frame_id_, tf2::TimePointZero);
            tf2::fromMsg(tf_base_cam.transform, T_base_cam_);
            tf_static_cached_ = true;
            update_transform_matrix();
            RCLCPP_INFO(this->get_logger(),
                "Static transform [%s -> %s] cached successfully!",
                parent_frame_.c_str(), frame_id_.c_str());
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Waiting for static TF [%s -> %s]: %s",
                parent_frame_.c_str(), frame_id_.c_str(), ex.what());
        }
    }
}

void TriangulationNode::grab(std::unique_ptr<const stereo_msgs::msg::DisparityImage> disp_msg)
{
    sensor_msgs::msg::Image::ConstSharedPtr left_img;
    {
        std::lock_guard<std::mutex> lock(left_img_mutex_);
        left_img = last_left_;
    }

    if (!left_img) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No left image available to colorize pointcloud");
        return;
    }

    // Dynamic parameters
    sampling_factor_ = static_cast<float>(std::clamp(this->get_parameter("sampling_factor").as_double(), 0.01, 1.0));
    crop_factor_ = std::clamp(this->get_parameter("crop_factor").as_double(), 0.01, 1.0);
    max_dist_ = this->get_parameter("max_dist").as_double();
    min_disp_ = this->get_parameter("min_disp").as_double();
    use_gpu_ = this->get_parameter("use_gpu").as_bool();

    baseline_ = disp_msg->t;
    if (fx_ == 0.0f) {
        fx_ = disp_msg->f;
        fy_ = disp_msg->f;
    }
    if (principal_x_ == 0.0f) {
        principal_x_ = disp_msg->image.width * 0.5f;
        principal_y_ = disp_msg->image.height * 0.5f;
    }

    int width = static_cast<int>(disp_msg->image.width);
    int height = static_cast<int>(disp_msg->image.height);

    int step = std::max(1, static_cast<int>(1.0f / sampling_factor_));
    int crop_width = static_cast<int>(width * crop_factor_);
    int crop_height = static_cast<int>(height * crop_factor_);
    int u0 = (width - crop_width) / 2;
    int v0 = (height - crop_height) / 2;
    int u1 = u0 + crop_width;
    int v1 = v0 + crop_height;

    // Detect image encoding & channels
    int channels = 3;
    bool is_rgb = false;
    if (left_img->encoding == "mono8" || left_img->encoding == "8UC1") {
        channels = 1;
    } else if (left_img->encoding == "rgb8") {
        channels = 3;
        is_rgb = true;
    } else if (left_img->encoding == "bgr8") {
        channels = 3;
        is_rgb = false;
    } else if (left_img->encoding == "rgba8") {
        channels = 4;
        is_rgb = true;
    } else if (left_img->encoding == "bgra8") {
        channels = 4;
        is_rgb = false;
    } else {
        // Fallback: estimate from step / width
        channels = (width > 0) ? std::max(1, static_cast<int>(left_img->step / width)) : 3;
    }

    // Apply CLAHE on L channel of Lab color space (local to this frame)
    cv::Mat clahe_img;
    const uint8_t* img_data = left_img->data.data();
    int img_step = static_cast<int>(left_img->step);

    if (apply_clahe_ && channels >= 3) {
        cv::Mat raw(height, width, (channels == 4) ? CV_8UC4 : CV_8UC3,
                    const_cast<uint8_t*>(left_img->data.data()),
                    left_img->step);
        cv::Mat lab;
        cv::cvtColor(raw, lab, is_rgb ? cv::COLOR_RGB2Lab : cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> lab_channels(3);
        cv::split(lab, lab_channels);
        clahe_->apply(lab_channels[0], lab_channels[0]);
        cv::merge(lab_channels, lab);
        cv::cvtColor(lab, clahe_img, is_rgb ? cv::COLOR_Lab2RGB : cv::COLOR_Lab2BGR);
        img_data = clahe_img.data;
        img_step = static_cast<int>(clahe_img.step[0]);
    }

    passive_stereo::TriangulationParams params;
    params.width = width;
    params.height = height;
    params.u0 = u0;
    params.v0 = v0;
    params.u1 = u1;
    params.v1 = v1;
    params.step = step;
    params.fx = fx_;
    params.fy = fy_;
    params.cx = principal_x_;
    params.cy = principal_y_;
    params.baseline = baseline_;
    params.min_disp = static_cast<float>(min_disp_);
    params.max_dist_sq = (max_dist_ > 0.0) ? static_cast<float>(max_dist_ * max_dist_) : -1.0f;
    std::memcpy(params.R, R_combined_, sizeof(float) * 9);
    std::memcpy(params.T, T_combined_, sizeof(float) * 3);
    params.channels = channels;
    params.is_rgb = is_rgb;
    params.img_step = img_step;
    params.disp_step = static_cast<int>(disp_msg->image.step);
    params.confidence_radius = this->get_parameter("confidence_radius").as_int();
    params.confidence_alpha = static_cast<float>(this->get_parameter("confidence_alpha").as_double());
    params.min_confidence = static_cast<float>(this->get_parameter("min_confidence").as_double());
    params.invert_x = false;
    params.invert_y = false;
    params.invert_z = false;

    int n_u = (u1 - u0 + step - 1) / step;
    int n_v = (v1 - v0 + step - 1) / step;
    size_t max_pts = static_cast<size_t>(n_u) * static_cast<size_t>(n_v);

    const float* disp_data = reinterpret_cast<const float*>(disp_msg->image.data.data());

    size_t valid_pts = 0;
    const passive_stereo::PointXYZRGB* point_ptr = nullptr;

    if (use_gpu_ && cuda_triangulator_ && cuda_triangulator_->is_available()) {
        // GPU path: triangulator stages inputs into its own pinned buffers and
        // writes results into its own pinned output buffer.
        // h_pinned receives a pointer directly into that buffer — no extra memcpy.
        passive_stereo::PointXYZRGB* h_pinned = nullptr;
        valid_pts = cuda_triangulator_->triangulate(
            disp_data,
            disp_msg->image.data.size(),
            img_data,
            left_img->data.size(),
            params,
            max_pts,
            &h_pinned);
        point_ptr = h_pinned;
    } else {
        // CPU fallback path: uses cpu_point_buffer_ (pageable heap allocation)
        if (cpu_point_buffer_.size() < max_pts) {
            cpu_point_buffer_.resize(max_pts);
        }
        valid_pts = triangulate_cpu(
            disp_data,
            params.disp_step,
            img_data,
            params.img_step,
            params,
            cpu_point_buffer_.data(),
            max_pts);
        point_ptr = cpu_point_buffer_.data();
    }

    // Build PointCloud2 message
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header = disp_msg->header;
    cloud->header.stamp = this->get_clock()->now();
    cloud->header.frame_id = (has_parent_ && tf_static_cached_) ? parent_frame_ : frame_id_;

    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::UINT32);
    cloud->point_step = 16;
    cloud->height = 1;
    cloud->width = static_cast<uint32_t>(valid_pts);
    cloud->row_step = cloud->width * cloud->point_step;
    cloud->is_dense = false;
    cloud->data.resize(cloud->row_step);

    // Single memcpy from pinned (GPU path) or heap (CPU path) → ROS message buffer.
    // For the GPU path this is a fast L3-hot copy from pinned memory (no page faults).
    if (valid_pts > 0 && point_ptr) {
        std::memcpy(cloud->data.data(), point_ptr, cloud->row_step);
    }

    pub_cloud_->publish(std::move(cloud));
}

size_t TriangulationNode::triangulate_cpu(
    const float* disp_data,
    int disp_step,
    const uint8_t* img_data,
    int img_step,
    const passive_stereo::TriangulationParams& params,
    passive_stereo::PointXYZRGB* out_points,
    size_t max_points)
{
#ifdef _OPENMP
    int num_threads = omp_get_max_threads();
#else
    int num_threads = 1;
#endif
    std::vector<std::vector<passive_stereo::PointXYZRGB>> thread_buffers(num_threads);

#ifdef _OPENMP
    #pragma omp parallel
#endif
    {
#ifdef _OPENMP
        int tid = omp_get_thread_num();
#else
        int tid = 0;
#endif
        auto& local_pts = thread_buffers[tid];
        local_pts.clear();
        local_pts.reserve(max_points / num_threads);

        #pragma omp for schedule(static)
        for (int v = params.v0; v < params.v1; v += params.step) {
            const float* disp_row = reinterpret_cast<const float*>(
                reinterpret_cast<const char*>(disp_data) + v * disp_step);
            const uint8_t* img_row = img_data + v * img_step;

            for (int u = params.u0; u < params.u1; u += params.step) {
                float d = disp_row[u];
                if (d > params.min_disp) {
                    float b = std::abs(params.baseline);
                    float Z = b * params.fx / d;
                    float X = (static_cast<float>(u) - params.cx) * Z / params.fx;
                    float Y = (static_cast<float>(v) - params.cy) * Z / params.fy;
                    if (params.invert_x) X = -X;
                    if (params.invert_y) Y = -Y;
                    if (params.invert_z) Z = -Z;

                    float x_trans = params.R[0] * X + params.R[1] * Y + params.R[2] * Z + params.T[0];
                    float y_trans = params.R[3] * X + params.R[4] * Y + params.R[5] * Z + params.T[1];
                    float z_trans = params.R[6] * X + params.R[7] * Y + params.R[8] * Z + params.T[2];

                    if (params.max_dist_sq > 0.0f) {
                        float dist_sq = x_trans * x_trans + y_trans * y_trans + z_trans * z_trans;
                        if (dist_sq > params.max_dist_sq) {
                            continue;
                        }
                    }

                    // Confidence-based noise gate
                    if (params.confidence_radius > 0) {
                        float sum = 0.0f, sum_sq = 0.0f;
                        int count = 0;
                        for (int dv = -params.confidence_radius; dv <= params.confidence_radius; ++dv) {
                            int vv = v + dv;
                            if (vv < 0 || vv >= params.height) continue;
                            const float* nb_row = reinterpret_cast<const float*>(
                                reinterpret_cast<const char*>(disp_data) + vv * disp_step);
                            for (int du = -params.confidence_radius; du <= params.confidence_radius; ++du) {
                                int uu = u + du;
                                if (uu < 0 || uu >= params.width) continue;
                                float dn = nb_row[uu];
                                if (dn > params.min_disp) {
                                    sum += dn;
                                    sum_sq += dn * dn;
                                    count++;
                                }
                            }
                        }
                        if (count > 1) {
                            float mean = sum / count;
                            float variance = (sum_sq / count) - (mean * mean);
                            float sigma = std::sqrt(std::max(variance, 0.0f));
                            float conf = 1.0f / (1.0f + params.confidence_alpha * sigma);
                            if (conf < params.min_confidence) {
                                continue;
                            }
                        }
                    }

                    uint32_t rgb = 0;
                    if (params.channels == 1) {
                        uint8_t gray = img_row[u];
                        rgb = (static_cast<uint32_t>(gray) << 16) |
                              (static_cast<uint32_t>(gray) << 8) |
                              static_cast<uint32_t>(gray);
                    } else if (params.channels == 3) {
                        uint8_t c0 = img_row[u * 3 + 0];
                        uint8_t c1 = img_row[u * 3 + 1];
                        uint8_t c2 = img_row[u * 3 + 2];
                        if (params.is_rgb) {
                            rgb = (static_cast<uint32_t>(c0) << 16) |
                                  (static_cast<uint32_t>(c1) << 8) |
                                  static_cast<uint32_t>(c2);
                        } else { // BGR
                            rgb = (static_cast<uint32_t>(c2) << 16) |
                                  (static_cast<uint32_t>(c1) << 8) |
                                  static_cast<uint32_t>(c0);
                        }
                    } else if (params.channels == 4) {
                        uint8_t c0 = img_row[u * 4 + 0];
                        uint8_t c1 = img_row[u * 4 + 1];
                        uint8_t c2 = img_row[u * 4 + 2];
                        if (params.is_rgb) {
                            rgb = (static_cast<uint32_t>(c0) << 16) |
                                  (static_cast<uint32_t>(c1) << 8) |
                                  static_cast<uint32_t>(c2);
                        } else { // BGRA
                            rgb = (static_cast<uint32_t>(c2) << 16) |
                                  (static_cast<uint32_t>(c1) << 8) |
                                  static_cast<uint32_t>(c0);
                        }
                    }

                    local_pts.push_back({x_trans, y_trans, z_trans, rgb});
                }
            }
        }
    }

    size_t offset = 0;
    for (const auto& local : thread_buffers) {
        size_t count = std::min(local.size(), max_points - offset);
        if (count > 0) {
            std::memcpy(out_points + offset, local.data(), count * sizeof(passive_stereo::PointXYZRGB));
            offset += count;
        }
        if (offset >= max_points) break;
    }

    return offset;
}


RCLCPP_COMPONENTS_REGISTER_NODE(TriangulationNode)