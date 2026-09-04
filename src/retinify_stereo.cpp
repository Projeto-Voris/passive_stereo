#include "retinify_stereo.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <cuda_runtime.h>

namespace passive_stereo
{

RetinifyStereoNode::RetinifyStereoNode(const rclcpp::NodeOptions & options)
: Node("retinify_stereo_node", options)
{
    // Parameter declarations
    this->declare_parameter<bool>("publish_disparity", true);
    this->declare_parameter<bool>("publish_pointcloud", true);
    this->declare_parameter<bool>("publish_depth", false);
    this->declare_parameter<bool>("publish_rectified", false);
    this->declare_parameter<bool>("debug_image", false);
    this->declare_parameter<bool>("use_exact_sync", false);
    this->declare_parameter<bool>("clahe", false);
    this->declare_parameter<bool>("use_gpu", true);
    this->declare_parameter<bool>("publish_confidence_field", true);
    this->declare_parameter<int>("confidence_radius", 2);
    this->declare_parameter<double>("confidence_alpha", 2.0);
    this->declare_parameter<double>("min_confidence", 0.35);
    this->declare_parameter<std::string>("depth_mode", "accurate");
    this->declare_parameter<std::string>("calibration_file", "");
    this->declare_parameter<std::string>("frame_id", "left_camera_link");
    this->declare_parameter<std::string>("parent_frame", "");
    this->declare_parameter<bool>("invert_x", false);
    this->declare_parameter<bool>("invert_y", false);
    this->declare_parameter<bool>("invert_z", false);
    this->declare_parameter<double>("sampling_factor", 1.0);
    this->declare_parameter<double>("crop_factor", 1.0);
    this->declare_parameter<double>("min_disp", 1.0);
    this->declare_parameter<double>("max_dist", 15.0);
    this->declare_parameter<int>("sync_queue_size", 10);

    // Get initial parameters
    publish_disparity_ = this->get_parameter("publish_disparity").as_bool();
    publish_pointcloud_ = this->get_parameter("publish_pointcloud").as_bool();
    publish_depth_ = this->get_parameter("publish_depth").as_bool();
    publish_rectified_ = this->get_parameter("publish_rectified").as_bool();
    debug_image_ = this->get_parameter("debug_image").as_bool();
    use_exact_sync_ = this->get_parameter("use_exact_sync").as_bool();
    apply_clahe_ = this->get_parameter("clahe").as_bool();
    use_gpu_ = this->get_parameter("use_gpu").as_bool();
    publish_confidence_field_ = this->get_parameter("publish_confidence_field").as_bool();
    confidence_radius_ = this->get_parameter("confidence_radius").as_int();
    confidence_alpha_ = this->get_parameter("confidence_alpha").as_double();
    min_confidence_ = this->get_parameter("min_confidence").as_double();
    invert_x_ = this->get_parameter("invert_x").as_bool();
    invert_y_ = this->get_parameter("invert_y").as_bool();
    invert_z_ = this->get_parameter("invert_z").as_bool();
    depth_mode_str_ = this->get_parameter("depth_mode").as_string();
    calibration_file_ = this->get_parameter("calibration_file").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    parent_frame_ = this->get_parameter("parent_frame").as_string();
    sampling_factor_ = this->get_parameter("sampling_factor").as_double();
    crop_factor_ = this->get_parameter("crop_factor").as_double();
    min_disp_ = this->get_parameter("min_disp").as_double();
    max_dist_ = this->get_parameter("max_dist").as_double();
    sync_queue_size_ = this->get_parameter("sync_queue_size").as_int();

    if (apply_clahe_) {
        clahe_->setClipLimit(5.0);
        clahe_->setTilesGridSize(cv::Size(5, 5));
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    updateTransformMatrix();

    // Initialize CUDA Triangulator
    cuda_triangulator_ = std::make_unique<passive_stereo::CudaTriangulator>();
    if (use_gpu_ && cuda_triangulator_->is_available()) {
        RCLCPP_INFO(this->get_logger(),
            "RetinifyStereoNode: GPU CUDA Triangulation & Confidence Filter enabled on [%s]",
            cuda_triangulator_->get_device_name().c_str());
    } else {
        RCLCPP_INFO(this->get_logger(),
            "RetinifyStereoNode: Multi-threaded OpenMP CPU Triangulation enabled");
    }

    // QoS Setup
    auto sensor_qos = rclcpp::SensorDataQoS();
    rclcpp::QoS debug_qos(2);
    debug_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    debug_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    // Subscriptions for CameraInfo
    sub_left_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "left/camera_info", sensor_qos,
        std::bind(&RetinifyStereoNode::onCameraInfoLeft, this, std::placeholders::_1), sub_options);

    sub_right_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "right/camera_info", sensor_qos,
        std::bind(&RetinifyStereoNode::onCameraInfoRight, this, std::placeholders::_1), sub_options);

    // Subscriptions for stereo images
    sub_left_img_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "left/image_rect", sensor_qos.get_rmw_qos_profile(), sub_options);
    sub_right_img_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "right/image_rect", sensor_qos.get_rmw_qos_profile(), sub_options);

    if (use_exact_sync_) {
        exact_sync_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
            ExactSyncPolicy(sync_queue_size_), *sub_left_img_, *sub_right_img_);
        exact_sync_->registerCallback(
            std::bind(&RetinifyStereoNode::onStereoImages, this, std::placeholders::_1, std::placeholders::_2));
    } else {
        approx_sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
            ApproximateSyncPolicy(sync_queue_size_), *sub_left_img_, *sub_right_img_);
        approx_sync_->registerCallback(
            std::bind(&RetinifyStereoNode::onStereoImages, this, std::placeholders::_1, std::placeholders::_2));
    }

    // Publishers
    if (publish_disparity_) {
        pub_disparity_ = this->create_publisher<stereo_msgs::msg::DisparityImage>("disparity/image", sensor_qos);
    }
    if (publish_pointcloud_) {
        pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("disparity/pointcloud", sensor_qos);
    }
    if (publish_depth_) {
        pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>("depth/image", sensor_qos);
    }
    if (publish_rectified_) {
        pub_rect_left_ = this->create_publisher<sensor_msgs::msg::Image>("left/image_rect", sensor_qos);
        pub_rect_right_ = this->create_publisher<sensor_msgs::msg::Image>("right/image_rect", sensor_qos);
    }
    if (debug_image_) {
        pub_debug_disp_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("disparity/debug/image", debug_qos);
    }

    RCLCPP_INFO(this->get_logger(),
        "RetinifyStereoNode initialized. Waiting for camera calibration and images...");
}

RetinifyStereoNode::~RetinifyStereoNode()
{
    if (h_pinned_disp_) {
        cudaFreeHost(h_pinned_disp_);
        h_pinned_disp_ = nullptr;
        pinned_disp_bytes_ = 0;
    }
}

void RetinifyStereoNode::updateTransformMatrix()
{
    parent_frame_ = this->get_parameter("parent_frame").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    if (!parent_frame_.empty() && !tf_static_cached_) {
        try {
            auto tf_base_cam = tf_buffer_->lookupTransform(
                parent_frame_, frame_id_, tf2::TimePointZero);
            tf2::fromMsg(tf_base_cam.transform, T_base_cam_);
            tf_static_cached_ = true;

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

            RCLCPP_INFO(this->get_logger(),
                "Cached static TF transform [%s -> %s]",
                parent_frame_.c_str(), frame_id_.c_str());
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Waiting for TF [%s -> %s]: %s", parent_frame_.c_str(), frame_id_.c_str(), ex.what());
        }
    }
}

void RetinifyStereoNode::onCameraInfoLeft(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(calib_mutex_);
    if (left_info_received_) return;
    left_camera_info_ = *msg;
    left_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Left CameraInfo received: %ux%u", msg->width, msg->height);
}

void RetinifyStereoNode::onCameraInfoRight(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(calib_mutex_);
    if (right_info_received_) return;
    right_camera_info_ = *msg;

    // Use projection matrix P if available, else intrinsic matrix K
    if (msg->p[0] > 0.0) {
        fx_ = msg->p[0];
        fy_ = msg->p[5];
        cx_ = msg->p[2];
        cy_ = msg->p[6];
        double tx = msg->p[3];
        baseline_ = std::abs(tx / fx_);
    } else {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        baseline_ = 0.1; // Default fallback if not defined in P
    }

    right_info_received_ = true;
    RCLCPP_INFO(this->get_logger(),
        "Right CameraInfo received. fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f, baseline: %.4fm",
        fx_, fy_, cx_, cy_, baseline_);
}

bool RetinifyStereoNode::initializePipeline(uint32_t width, uint32_t height)
{
    retinify::DepthMode depth_mode = retinify::DepthMode::ACCURATE;
    if (depth_mode_str_ == "fast") {
        depth_mode = retinify::DepthMode::FAST;
    } else if (depth_mode_str_ == "balanced") {
        depth_mode = retinify::DepthMode::BALANCED;
    } else {
        depth_mode = retinify::DepthMode::ACCURATE;
    }

    retinify::CalibrationParameters calib{};
    if (!calibration_file_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Loading calibration parameters from file: %s", calibration_file_.c_str());
        auto status = retinify::LoadCalibrationParameters(calibration_file_.c_str(), calib);
        if (!status.IsOK()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load calibration parameters from %s", calibration_file_.c_str());
            return false;
        }
    } else {
        calib.imageWidth = width;
        calib.imageHeight = height;

        calib.leftIntrinsics.fx = fx_;
        calib.leftIntrinsics.fy = fy_;
        calib.leftIntrinsics.cx = cx_;
        calib.leftIntrinsics.cy = cy_;

        calib.rightIntrinsics = calib.leftIntrinsics;
        calib.rotation = retinify::Identity();
        calib.translation = {-std::abs(baseline_), 0.0, 0.0};
    }

    auto status = pipeline_.Initialize(
        width, height, retinify::PixelFormat::RGB8, depth_mode, calib);

    if (!status.IsOK()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Retinify pipeline!");
        return false;
    }

    // Allocate pinned host memory for disparity
    size_t needed_disp_bytes = width * height * sizeof(float);
    if (pinned_disp_bytes_ < needed_disp_bytes) {
        if (h_pinned_disp_) cudaFreeHost(h_pinned_disp_);
        cudaError_t err = cudaHostAlloc(&h_pinned_disp_, needed_disp_bytes, cudaHostAllocDefault);
        if (err != cudaSuccess) {
            RCLCPP_WARN(this->get_logger(), "cudaHostAlloc failed for disparity, falling back to pageable host memory");
            h_pinned_disp_ = nullptr;
            cpu_disp_buffer_.resize(width * height);
            pinned_disp_bytes_ = 0;
        } else {
            pinned_disp_bytes_ = needed_disp_bytes;
        }
    }

    if (publish_depth_) {
        depth_buffer_.resize(width * height);
    }
    if (publish_rectified_) {
        rect_left_buffer_.resize(width * height * 3);
        rect_right_buffer_.resize(width * height * 3);
    }

    pipeline_initialized_ = true;
    RCLCPP_INFO(this->get_logger(),
        "Retinify stereo pipeline initialized: %ux%u (DepthMode=%s, GPU Triangulator=%s, PinnedDisp=%s)",
        width, height, depth_mode_str_.c_str(),
        (use_gpu_ && cuda_triangulator_->is_available()) ? "YES" : "CPU",
        (h_pinned_disp_ != nullptr) ? "YES" : "NO");
    return true;
}

cv::Mat RetinifyStereoNode::applyCLAHE(const cv::Mat & input_bgr)
{
    cv::Mat lab_image;
    cv::cvtColor(input_bgr, lab_image, cv::COLOR_RGB2Lab);

    std::vector<cv::Mat> lab_channels(3);
    cv::split(lab_image, lab_channels);

    clahe_->apply(lab_channels[0], lab_channels[0]);

    cv::Mat processed_lab;
    cv::merge(lab_channels, processed_lab);

    cv::Mat output_bgr;
    cv::cvtColor(processed_lab, output_bgr, cv::COLOR_Lab2RGB);

    return output_bgr;
}

size_t RetinifyStereoNode::triangulateCPU(
    const float* disp_data,
    int disp_step,
    const uint8_t* img_data,
    int img_step,
    const TriangulationParams& params,
    size_t max_points,
    void* out_points,
    bool with_confidence)
{
#ifdef _OPENMP
    int num_threads = omp_get_max_threads();
#else
    int num_threads = 1;
#endif

    std::vector<std::vector<PointXYZRGBConfidence>> thread_conf_buf(num_threads);
    std::vector<std::vector<PointXYZRGB>> thread_buf(num_threads);

#ifdef _OPENMP
    #pragma omp parallel
#endif
    {
#ifdef _OPENMP
        int tid = omp_get_thread_num();
#else
        int tid = 0;
#endif
        if (with_confidence) {
            thread_conf_buf[tid].clear();
            thread_conf_buf[tid].reserve(max_points / num_threads);
        } else {
            thread_buf[tid].clear();
            thread_buf[tid].reserve(max_points / num_threads);
        }

#ifdef _OPENMP
        #pragma omp for schedule(static)
#endif
        for (int v = params.v0; v < params.v1; v += params.step) {
            const float* disp_row = reinterpret_cast<const float*>(
                reinterpret_cast<const char*>(disp_data) + v * disp_step);
            const uint8_t* img_row = img_data + v * img_step;

            for (int u = params.u0; u < params.u1; u += params.step) {
                float d = disp_row[u];
                if (d <= params.min_disp) continue;

                float b = std::abs(params.baseline);
                float Z = b * params.fx / d;
                float X = (static_cast<float>(u) - params.cx) * Z / params.fx;
                float Y = (static_cast<float>(v) - params.cy) * Z / params.fy;

                if (params.invert_x) X = -X;
                if (params.invert_y) Y = -Y;
                if (params.invert_z) Z = -Z;

                if (Z <= 0.0f && !params.invert_z) continue;

                float x_trans = params.R[0] * X + params.R[1] * Y + params.R[2] * Z + params.T[0];
                float y_trans = params.R[3] * X + params.R[4] * Y + params.R[5] * Z + params.T[1];
                float z_trans = params.R[6] * X + params.R[7] * Y + params.R[8] * Z + params.T[2];

                if (params.max_dist_sq > 0.0f) {
                    float dist_sq = x_trans * x_trans + y_trans * y_trans + z_trans * z_trans;
                    if (dist_sq > params.max_dist_sq) continue;
                }

                float conf = 1.0f;
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
                        conf = 1.0f / (1.0f + params.confidence_alpha * sigma);
                        if (conf < params.min_confidence) {
                            continue;
                        }
                    } else {
                        continue;
                    }
                }

                uint32_t rgb = 0;
                if (params.channels >= 3) {
                    uint8_t c0 = img_row[u * 3 + 0];
                    uint8_t c1 = img_row[u * 3 + 1];
                    uint8_t c2 = img_row[u * 3 + 2];
                    if (params.is_rgb) {
                        rgb = (static_cast<uint32_t>(c0) << 16) |
                              (static_cast<uint32_t>(c1) << 8) |
                              static_cast<uint32_t>(c2);
                    } else {
                        rgb = (static_cast<uint32_t>(c2) << 16) |
                              (static_cast<uint32_t>(c1) << 8) |
                              static_cast<uint32_t>(c0);
                    }
                }

                if (with_confidence) {
                    thread_conf_buf[tid].push_back({x_trans, y_trans, z_trans, rgb, conf});
                } else {
                    thread_buf[tid].push_back({x_trans, y_trans, z_trans, rgb});
                }
            }
        }
    }

    size_t offset = 0;
    if (with_confidence) {
        auto* dst = reinterpret_cast<PointXYZRGBConfidence*>(out_points);
        for (const auto& local : thread_conf_buf) {
            size_t count = std::min(local.size(), max_points - offset);
            if (count > 0) {
                std::memcpy(dst + offset, local.data(), count * sizeof(PointXYZRGBConfidence));
                offset += count;
            }
            if (offset >= max_points) break;
        }
    } else {
        auto* dst = reinterpret_cast<PointXYZRGB*>(out_points);
        for (const auto& local : thread_buf) {
            size_t count = std::min(local.size(), max_points - offset);
            if (count > 0) {
                std::memcpy(dst + offset, local.data(), count * sizeof(PointXYZRGB));
                offset += count;
            }
            if (offset >= max_points) break;
        }
    }

    return offset;
}

void RetinifyStereoNode::onStereoImages(
    const sensor_msgs::msg::Image::ConstSharedPtr msg_left,
    const sensor_msgs::msg::Image::ConstSharedPtr msg_right)
{
    // Fetch dynamic parameters
    this->get_parameter("publish_disparity", publish_disparity_);
    this->get_parameter("publish_pointcloud", publish_pointcloud_);
    this->get_parameter("publish_depth", publish_depth_);
    this->get_parameter("publish_rectified", publish_rectified_);
    this->get_parameter("debug_image", debug_image_);
    this->get_parameter("use_gpu", use_gpu_);
    this->get_parameter("publish_confidence_field", publish_confidence_field_);
    this->get_parameter("confidence_radius", confidence_radius_);
    this->get_parameter("confidence_alpha", confidence_alpha_);
    this->get_parameter("min_confidence", min_confidence_);
    this->get_parameter("invert_x", invert_x_);
    this->get_parameter("invert_y", invert_y_);
    this->get_parameter("invert_z", invert_z_);
    this->get_parameter("sampling_factor", sampling_factor_);
    this->get_parameter("crop_factor", crop_factor_);
    this->get_parameter("min_disp", min_disp_);
    this->get_parameter("max_dist", max_dist_);
    this->get_parameter("clahe", apply_clahe_);
    frame_id_ = this->get_parameter("frame_id").as_string();

    updateTransformMatrix();

    // Check calibration readiness
    if (calibration_file_.empty() && (!left_info_received_ || !right_info_received_)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "Waiting for camera info before running Retinify stereo pipeline...");
        return;
    }

    uint32_t width = msg_left->width;
    uint32_t height = msg_left->height;

    // Initialize pipeline if not yet initialized
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    if (!pipeline_initialized_) {
        if (!initializePipeline(width, height)) {
            return;
        }
    }

    // Convert ROS Image to RGB8 cv::Mat for Retinify
    cv_bridge::CvImageConstPtr cv_left, cv_right;
    try {
        cv_left = cv_bridge::toCvShare(msg_left, sensor_msgs::image_encodings::RGB8);
        cv_right = cv_bridge::toCvShare(msg_right, sensor_msgs::image_encodings::RGB8);
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion to RGB8 failed: %s", e.what());
        return;
    }

    cv::Mat left_img = cv_left->image;
    cv::Mat right_img = cv_right->image;

    if (apply_clahe_) {
        left_img = applyCLAHE(left_img);
        right_img = applyCLAHE(right_img);
    }

    // Execute Retinify stereo matching on GPU
    auto status = pipeline_.Execute(
        left_img.ptr<uint8_t>(), left_img.step[0],
        right_img.ptr<uint8_t>(), right_img.step[0]);

    if (!status.IsOK()) {
        RCLCPP_ERROR(this->get_logger(), "Retinify pipeline execution failed!");
        return;
    }

    rclcpp::Time stamp = msg_left->header.stamp;

    float* disp_ptr = h_pinned_disp_ ? h_pinned_disp_ : cpu_disp_buffer_.data();

    // 1. Retrieve Disparity (1-channel float only)
    if (publish_disparity_ || debug_image_ || publish_pointcloud_) {
        auto disp_status = pipeline_.RetrieveDisparity(
            disp_ptr, width * sizeof(float));

        if (!disp_status.IsOK()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to retrieve disparity from Retinify!");
            return;
        }

        if (publish_disparity_ && pub_disparity_) {
            auto disp_msg = std::make_unique<stereo_msgs::msg::DisparityImage>();
            disp_msg->header.stamp = stamp;
            disp_msg->header.frame_id = frame_id_;
            disp_msg->f = static_cast<float>(fx_);
            disp_msg->t = static_cast<float>(baseline_);
            disp_msg->min_disparity = static_cast<float>(min_disp_);
            disp_msg->max_disparity = 256.0f;

            disp_msg->image.header = disp_msg->header;
            disp_msg->image.height = height;
            disp_msg->image.width = width;
            disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            disp_msg->image.is_bigendian = false;
            disp_msg->image.step = width * sizeof(float);
            disp_msg->image.data.resize(disp_msg->image.step * height);
            std::memcpy(disp_msg->image.data.data(), disp_ptr, disp_msg->image.data.size());

            pub_disparity_->publish(std::move(disp_msg));
        }

        if (debug_image_ && pub_debug_disp_) {
            cv::Mat colored_disp(height, width, CV_8UC3);
            auto col_status = retinify::ColorizeDisparity(
                disp_ptr, width * sizeof(float),
                colored_disp.ptr<uint8_t>(), colored_disp.step[0],
                width, height, 256.0f);

            if (col_status.IsOK()) {
                cv::Mat resized_disp;
                cv::resize(colored_disp, resized_disp, cv::Size(), 0.5, 0.5);
                cv::cvtColor(resized_disp, resized_disp, cv::COLOR_RGB2BGR);

                std::vector<uchar> jpeg_buffer;
                std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};
                cv::imencode(".jpg", resized_disp, jpeg_buffer, params);

                auto debug_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                debug_msg->header.stamp = stamp;
                debug_msg->header.frame_id = frame_id_;
                debug_msg->format = "jpeg";
                debug_msg->data = std::move(jpeg_buffer);
                pub_debug_disp_->publish(std::move(debug_msg));
            }
        }
    }

    // 2. High-Efficiency GPU Triangulation & Confidence Filtering
    if (publish_pointcloud_ && pub_pointcloud_) {
        float sampling = static_cast<float>(std::clamp(sampling_factor_, 0.01, 1.0));
        int step = std::max(1, static_cast<int>(1.0f / sampling));
        double crop = std::clamp(crop_factor_, 0.01, 1.0);
        int crop_w = static_cast<int>(width * crop);
        int crop_h = static_cast<int>(height * crop);
        int u0 = (static_cast<int>(width) - crop_w) / 2;
        int v0 = (static_cast<int>(height) - crop_h) / 2;
        int u1 = u0 + crop_w;
        int v1 = v0 + crop_h;

        int n_u = (u1 - u0 + step - 1) / step;
        int n_v = (v1 - v0 + step - 1) / step;
        size_t max_pts = static_cast<size_t>(n_u) * static_cast<size_t>(n_v);

        TriangulationParams params;
        params.width = width;
        params.height = height;
        params.u0 = u0;
        params.v0 = v0;
        params.u1 = u1;
        params.v1 = v1;
        params.step = step;
        params.fx = static_cast<float>(fx_);
        params.fy = static_cast<float>(fy_);
        params.cx = static_cast<float>(cx_);
        params.cy = static_cast<float>(cy_);
        params.baseline = static_cast<float>(baseline_);
        params.min_disp = static_cast<float>(min_disp_);
        params.max_dist_sq = (max_dist_ > 0.0) ? static_cast<float>(max_dist_ * max_dist_) : -1.0f;
        std::memcpy(params.R, R_combined_, sizeof(float) * 9);
        std::memcpy(params.T, T_combined_, sizeof(float) * 3);
        params.channels = 3;
        params.is_rgb = true;
        params.img_step = static_cast<int>(left_img.step[0]);
        params.disp_step = width * sizeof(float);
        params.confidence_radius = confidence_radius_;
        params.confidence_alpha = static_cast<float>(confidence_alpha_);
        params.min_confidence = static_cast<float>(min_confidence_);
        params.invert_x = invert_x_;
        params.invert_y = invert_y_;
        params.invert_z = invert_z_;

        size_t valid_pts = 0;
        const void* point_ptr = nullptr;
        uint32_t point_step = publish_confidence_field_ ? sizeof(PointXYZRGBConfidence) : sizeof(PointXYZRGB);

        bool use_cuda = use_gpu_ && cuda_triangulator_ && cuda_triangulator_->is_available();

        if (use_cuda) {
            if (publish_confidence_field_) {
                PointXYZRGBConfidence* h_pts = nullptr;
                valid_pts = cuda_triangulator_->triangulate(
                    disp_ptr, width * height * sizeof(float),
                    left_img.ptr<uint8_t>(), left_img.step[0] * height,
                    params, max_pts, &h_pts);
                point_ptr = h_pts;
            } else {
                PointXYZRGB* h_pts = nullptr;
                valid_pts = cuda_triangulator_->triangulate(
                    disp_ptr, width * height * sizeof(float),
                    left_img.ptr<uint8_t>(), left_img.step[0] * height,
                    params, max_pts, &h_pts);
                point_ptr = h_pts;
            }
        } else {
            // CPU fallback path
            size_t needed_bytes = max_pts * point_step;
            if (cpu_point_buffer_.size() < needed_bytes) {
                cpu_point_buffer_.resize(needed_bytes);
            }
            valid_pts = triangulateCPU(
                disp_ptr, params.disp_step,
                left_img.ptr<uint8_t>(), params.img_step,
                params, max_pts, cpu_point_buffer_.data(),
                publish_confidence_field_);
            point_ptr = cpu_point_buffer_.data();
        }

        // Build sensor_msgs::msg::PointCloud2 message
        std::string effective_frame = frame_id_;
        if (effective_frame.empty() || effective_frame == "auto") {
            effective_frame = msg_left->header.frame_id;
        }

        auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header.stamp = stamp;
        cloud_msg->header.frame_id = (tf_static_cached_ && !parent_frame_.empty()) ? parent_frame_ : effective_frame;
        cloud_msg->height = 1;
        cloud_msg->width = static_cast<uint32_t>(valid_pts);
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        if (publish_confidence_field_) {
            modifier.setPointCloud2Fields(5,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "rgb", 1, sensor_msgs::msg::PointField::UINT32,
                "confidence", 1, sensor_msgs::msg::PointField::FLOAT32);
        } else {
            modifier.setPointCloud2Fields(4,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "rgb", 1, sensor_msgs::msg::PointField::UINT32);
        }

        cloud_msg->point_step = point_step;
        cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
        cloud_msg->data.resize(cloud_msg->row_step);

        if (valid_pts > 0 && point_ptr) {
            std::memcpy(cloud_msg->data.data(), point_ptr, cloud_msg->row_step);
        }

        pub_pointcloud_->publish(std::move(cloud_msg));
    }

    // 3. Depth Retrieval & Publishing
    if (publish_depth_ && pub_depth_) {
        auto depth_status = pipeline_.RetrieveDepth(
            depth_buffer_.data(), width * sizeof(float));

        if (depth_status.IsOK()) {
            auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
            depth_msg->header.stamp = stamp;
            depth_msg->header.frame_id = frame_id_;
            depth_msg->height = height;
            depth_msg->width = width;
            depth_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            depth_msg->is_bigendian = false;
            depth_msg->step = width * sizeof(float);
            depth_msg->data.resize(depth_msg->step * height);
            std::memcpy(depth_msg->data.data(), depth_buffer_.data(), depth_msg->data.size());
            pub_depth_->publish(std::move(depth_msg));
        }
    }

    // 4. Rectified Images Retrieval & Publishing
    if (publish_rectified_ && pub_rect_left_ && pub_rect_right_) {
        auto rect_status = pipeline_.RetrieveRectifiedImages(
            rect_left_buffer_.data(), width * 3,
            rect_right_buffer_.data(), width * 3);

        if (rect_status.IsOK()) {
            auto rect_left_msg = std::make_unique<sensor_msgs::msg::Image>();
            rect_left_msg->header.stamp = stamp;
            rect_left_msg->header.frame_id = frame_id_;
            rect_left_msg->height = height;
            rect_left_msg->width = width;
            rect_left_msg->encoding = sensor_msgs::image_encodings::RGB8;
            rect_left_msg->is_bigendian = false;
            rect_left_msg->step = width * 3;
            rect_left_msg->data.resize(rect_left_msg->step * height);
            std::memcpy(rect_left_msg->data.data(), rect_left_buffer_.data(), rect_left_msg->data.size());
            pub_rect_left_->publish(std::move(rect_left_msg));

            auto rect_right_msg = std::make_unique<sensor_msgs::msg::Image>();
            rect_right_msg->header.stamp = stamp;
            rect_right_msg->header.frame_id = frame_id_;
            rect_right_msg->height = height;
            rect_right_msg->width = width;
            rect_right_msg->encoding = sensor_msgs::image_encodings::RGB8;
            rect_right_msg->is_bigendian = false;
            rect_right_msg->step = width * 3;
            rect_right_msg->data.resize(rect_right_msg->step * height);
            std::memcpy(rect_right_msg->data.data(), rect_right_buffer_.data(), rect_right_msg->data.size());
            pub_rect_right_->publish(std::move(rect_right_msg));
        }
    }
}

} // namespace passive_stereo

RCLCPP_COMPONENTS_REGISTER_NODE(passive_stereo::RetinifyStereoNode)
