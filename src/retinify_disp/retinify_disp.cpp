#include "retinify_disp.hpp"

RetinifyDisparityNode::RetinifyDisparityNode(const rclcpp::NodeOptions & options)
: Node("retinify_disparity_ipc", options)
{
    this->declare_parameter<bool>("publish_rectified", false);
    this->declare_parameter<bool>("debug_image", false);
    this->declare_parameter<bool>("publish_disp", true);
    this->get_parameter("publish_disp", publish_disp);
    this->get_parameter("publish_rectified", publish_rectified_);
    this->get_parameter("debug_image", debug_image_);

    rclcpp::QoS debug_qos_profile(2);
    rclcpp::QoS subscribe_qos_profile(5);

    debug_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    debug_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    subscribe_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);


    // 1. Camera Info Subscribers (Needed to build rectification maps)
    left_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "left/camera_info", subscribe_qos_profile, std::bind(&RetinifyDisparityNode::grabcamInfoLeft, this, std::placeholders::_1));
    right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "right/camera_info", subscribe_qos_profile, std::bind(&RetinifyDisparityNode::grabcamInfoRight, this, std::placeholders::_1));

    // 2. Image Subscribers with Synchronization
    left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "left/image_raw", subscribe_qos_profile.get_rmw_qos_profile());
    right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "right/image_raw", subscribe_qos_profile.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
                approximate_sync_policy(10), *left_sub_, *right_sub_);
    sync_->registerCallback(std::bind(&RetinifyDisparityNode::grabStereo, this, std::placeholders::_1, std::placeholders::_2));

    // 3. Publishers
    if(publish_disp){
        pub_disp_ = this->create_publisher<stereo_msgs::msg::DisparityImage>("disparity/image", 10);
    }
    if (publish_rectified_) {
        rect_left_publisher = this->create_publisher<sensor_msgs::msg::Image>("left/rect_image", 10);
        rect_right_publisher = this->create_publisher<sensor_msgs::msg::Image>("right/rect_image", 10);
    }
    if (debug_image_){
        debug_disp_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("disparity/debug/image", debug_qos_profile);
    }
}

void RetinifyDisparityNode::grabcamInfoLeft(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (left_info_received_) return;
    left_camera_info_ = *msg;
    left_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Left Camera Info received.");
}

void RetinifyDisparityNode::grabcamInfoRight(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (right_info_received_) return;
    right_camera_info_ = *msg;
    right_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Right Camera Info received.");
}

void RetinifyDisparityNode::grabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr msgRight){
    // GUARD: Only proceed if we have both CameraInfos
    this->get_parameter("publish_disp", publish_disp);
    this->get_parameter("publish_rectified", publish_rectified_);
    this->get_parameter("debug_image", debug_image_);
    if (!left_info_received_ || !right_info_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for Camera Info...");
        return; 
    }

    // Initialize Maps and Pipeline ONCE
    if (!maps_initialized_) {
        // if (!CalculateRectificationRemaps()) return;
        if (!retinifyCalibParam()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert calibration parameters for Retinify.");
            return;
        }
        auto status = pipeline.Initialize(static_cast<std::uint32_t>(left_camera_info_.width),
                                         static_cast<std::uint32_t>(left_camera_info_.height),
                                        retinify::PixelFormat::RGB8, 
                                        retinify::DepthMode::ACCURATE,
                                        calib_);
        if (!status.IsOK()) {
            RCLCPP_ERROR(this->get_logger(), "Pipeline init failed");
            return;
        }
        maps_initialized_ = true;
    }

    // Conversion
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try {
        cv_ptrLeft  = cv_bridge::toCvShare(msgLeft, msgLeft->encoding);
        cv_ptrRight = cv_bridge::toCvShare(msgRight, msgRight->encoding);

    } 
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
        return;
    }

    // Retinify Execution
    if (pipeline.Execute(cv_ptrLeft->image.ptr<uint8_t>(), cv_ptrLeft->image.step[0], 
                         cv_ptrRight->image.ptr<uint8_t>(), cv_ptrRight->image.step[0]).IsOK()) {
        cv::Mat leftRectifiedImg = cv::Mat::zeros(cv_ptrLeft->image.size(), cv_ptrLeft->image.type());
        cv::Mat rightRectifiedImg = cv::Mat::zeros(cv_ptrRight->image.size(), cv_ptrRight->image.type());
        if(pipeline.RetrieveRectifiedLeftImage(leftRectifiedImg.ptr<std::uint8_t>(), leftRectifiedImg.step[0]).IsOK())
        {
            pipeline.RetrieveRectifiedRightImage(rightRectifiedImg.ptr<std::uint8_t>(), rightRectifiedImg.step[0]);
            if(publish_rectified_){
                auto rect_left_msg = std::make_unique<sensor_msgs::msg::Image>();
                auto rect_right_msg = std::make_unique<sensor_msgs::msg::Image>();

                rect_left_msg->header = msgLeft->header;
                rect_left_msg->header.stamp = this->get_clock()->now();
                rect_left_msg->encoding = "BGR8";
                rect_left_msg->height = leftRectifiedImg.rows;
                rect_left_msg->width = leftRectifiedImg.cols;
                rect_left_msg->step = leftRectifiedImg.cols * 3; // 3 channels
                rect_left_msg->data.assign(leftRectifiedImg.data, leftRectifiedImg.data + (leftRectifiedImg.cols * leftRectifiedImg.rows * 3));

                rect_right_msg->header = msgRight->header;
                rect_right_msg->header.stamp = this->get_clock()->now();
                rect_right_msg->encoding = "BGR8";
                rect_right_msg->height = rightRectifiedImg.rows;
                rect_right_msg->width = rightRectifiedImg.cols;
                rect_right_msg->step = rightRectifiedImg.cols * 3; // 3 channels
                rect_right_msg->data.assign(rightRectifiedImg.data, rightRectifiedImg.data + (rightRectifiedImg.cols * rightRectifiedImg.rows * 3));

                rect_left_publisher->publish(std::move(rect_left_msg));
                rect_right_publisher->publish(std::move(rect_right_msg));
            }

        cv::Mat disparity = cv::Mat::zeros(cv_ptrLeft->image.size(), CV_32FC1);
            // Retinify disparity retrieval
            if (pipeline.RetrieveDisparity(disparity.ptr<float>(), disparity.step[0]).IsOK()) {
                if(publish_disp){
                    // Build Disparity Message
                    auto disp_msg = std::make_unique<stereo_msgs::msg::DisparityImage>();
                    disp_msg->header = msgLeft->header;
                    disp_msg->header.stamp = this->get_clock()->now();
                    disp_msg->f = focal_length_;
                    disp_msg->t = baseline_;
                    disp_msg->min_disparity = 0;
                    disp_msg->max_disparity = 256;
                    
                    auto disp_img = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", disparity).toImageMsg();
                    disp_msg->image = *disp_img;
                    pub_disp_->publish(std::move(disp_msg));
                }
                if (debug_image_) {
                    cv::Mat disparityColored;
                    if(retinify::ColorizeDisparity(disparity.ptr<float>(), disparity.step[0], disparityColored.ptr<uint8_t>(), disparityColored.step[0], disparity.cols, disparity.rows, 256.0F).IsOK()) {
                        cv::resize(disparityColored, disparityColored, cv::Size(), 0.5, 0.5); 
                        cv::cvtColor(disparityColored, disparityColored, cv::COLOR_RGB2BGR);

                    cv_bridge::CvImage debug_img(msgLeft->header, "bgr8", disparityColored);
                    // 3. Compressão Manual (JPEG)
                    std::vector<uchar> buffer;
                    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 40};
                    
                    // Codifica a matriz para o buffer de bytes
                    cv::imencode(".jpg", disparityColored, buffer, compression_params);
                    auto debug_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                    debug_msg->header = msgLeft->header;
                    debug_msg->header.stamp = this->get_clock()->now();
                    debug_msg->format = "jpg";
                    debug_msg->data = buffer;
                    debug_disp_publisher->publish(std::move(debug_msg));
                    }
                    else{
                        RCLCPP_ERROR(this->get_logger(), "Failed to colorize disparity for debug image.");
                    }
                }
                else{
                    RCLCPP_ERROR(this->get_logger(), "Failed to retrieve disparity from Retinify.");
                    return;
                }
                cv::Mat pointcloud = cv::Mat::zeros(cv_ptrLeft->image.size(), CV_32FC3);
                if(pipeline.RetrievePointCloud(pointcloud.ptr<float>(), pointcloud.step[0]).IsOK()) {
                    publishColoredPointCloud(std::vector<float>(pointcloud.begin<float>(), pointcloud.end<float>()), leftRectifiedImg, msgLeft->header);
                }
                else{
                    RCLCPP_ERROR(this->get_logger(), "Failed to retrieve point cloud from Retinify.");
                    return;
                }
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Failed to retrieved rect left image");
                return;
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Pipeline execution failed");
            return;
        }
    }
}

void RetinifyDisparityNode::publishColoredPointCloud( const std::vector<float>& points,
                                                        const cv::Mat& color_img,
                                                        const std_msgs::msg::Header& header){
    uint32_t width = color_img.cols;
    uint32_t height = color_img.rows;
    auto pc_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

    pc_msg->header = header;
    pc_msg->height = height;
    pc_msg->width = width;
    pc_msg->is_dense = false;
    pc_msg->is_bigendian = false;

    // Define Fields: x, y, z, and rgb
    sensor_msgs::msg::PointField f_x, f_y, f_z, f_rgb;
    f_x.name = "x"; f_x.offset = 0; f_x.datatype = 7; f_x.count = 1; // FLOAT32
    f_y.name = "y"; f_y.offset = 4; f_y.datatype = 7; f_y.count = 1;
    f_z.name = "z"; f_z.offset = 8; f_z.datatype = 7; f_z.count = 1;
    f_rgb.name = "rgb"; f_rgb.offset = 12; f_rgb.datatype = 7; f_rgb.count = 1;

    pc_msg->fields = {f_x, f_y, f_z, f_rgb};
    pc_msg->point_step = 16; // 4 floats * 4 bytes
    pc_msg->row_step = pc_msg->point_step * width;
    pc_msg->data.resize(pc_msg->row_step * height);

    uint8_t* ptr = pc_msg->data.data();

    for (uint32_t i = 0; i < width * height; ++i) {
        // Copy XYZ (12 bytes)
        std::memcpy(ptr + (i * 16), &points[i * 3], 12);

        // Pack BGR into a single float for the "rgb" field
        // Note: OpenCV uses BGR, but PointCloud2 RGB field usually expects 0x00RRGGBB
        uint8_t b = color_img.data[i * 3 + 0];
        uint8_t g = color_img.data[i * 3 + 1];
        uint8_t r = color_img.data[i * 3 + 2];
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 | 
                        static_cast<uint32_t>(g) << 8  | 
                        static_cast<uint32_t>(b));
        
        std::memcpy(ptr + (i * 16) + 12, &rgb, 4);
    }

    pc_pub_->publish(std::move(pc_msg));
}

void RetinifyDisparityNode::RectifyImages(const cv::Mat& imgL, const cv::Mat& imgR, 
                                          const sensor_msgs::msg::Image::ConstSharedPtr msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr msgRight){
    // Simplified: Assuming input is handled or converted
    cv::remap(imgL, rectImgL, left_map1, left_map2, cv::INTER_LINEAR);
    cv::remap(imgR, rectImgR, right_map1, right_map2, cv::INTER_LINEAR);

    if (publish_rectified_) {
        auto out_left = std::make_unique<sensor_msgs::msg::Image>();
        // Fill and publish...
    }
}

bool RetinifyDisparityNode::CalculateRectificationRemaps() {
    try {
        cv::Mat K1(3, 3, CV_64F, left_camera_info_.k.data());
        cv::Mat D1(left_camera_info_.d.size(), 1, CV_64F, left_camera_info_.d.data());
        cv::Mat R1(3, 3, CV_64F, left_camera_info_.r.data());
        cv::Mat P1(3, 4, CV_64F, left_camera_info_.p.data());

        cv::Mat K2(3, 3, CV_64F, right_camera_info_.k.data());
        cv::Mat D2(right_camera_info_.d.size(), 1, CV_64F, right_camera_info_.d.data());
        cv::Mat R2(3, 3, CV_64F, right_camera_info_.r.data());
        cv::Mat P2(3, 4, CV_64F, right_camera_info_.p.data());

        cv::Size size(left_camera_info_.width, left_camera_info_.height);

        focal_length_ = P1.at<double>(0, 0);
        // baseline = -Tx / f
        baseline_ = std::abs(P2.at<double>(0, 3) / P2.at<double>(0, 0));

        cv::initUndistortRectifyMap(K1, D1, R1, P1, size, CV_32FC1, left_map1, left_map2);
        cv::initUndistortRectifyMap(K2, D2, R2, P2, size, CV_32FC1, right_map1, right_map2);
        
        return true;
    } catch (...) {
        return false;
    }

}

bool RetinifyDisparityNode::retinifyCalibParam(){
        // 1. Map Intrinsics (K) and Distortion (D)
    try {
        calib_.leftIntrinsics.fx = static_cast<float>(left_camera_info_.k[0]);
        calib_.leftIntrinsics.fy = static_cast<float>(left_camera_info_.k[4]);
        calib_.leftIntrinsics.cx = static_cast<float>(left_camera_info_.k[2]);
        calib_.leftIntrinsics.cy = static_cast<float>(left_camera_info_.k[5]);
        calib_.leftDistortion.k1 = static_cast<float>(left_camera_info_.d[0]);
        calib_.leftDistortion.k2 = static_cast<float>(left_camera_info_.d[1]);
        calib_.leftDistortion.p1 = static_cast<float>(left_camera_info_.d[2]);
        calib_.leftDistortion.p2 = static_cast<float>(left_camera_info_.d[3]);
        calib_.leftDistortion.k3 = static_cast<float>(left_camera_info_.d[4]);

        calib_.rightIntrinsics.fx = static_cast<float>(right_camera_info_.k[0]);
        calib_.rightIntrinsics.fy = static_cast<float>(right_camera_info_.k[4]);
        calib_.rightIntrinsics.cx = static_cast<float>(right_camera_info_.k[2]);
        calib_.rightIntrinsics.cy = static_cast<float>(right_camera_info_.k[5]);
        calib_.rightDistortion.k1 = static_cast<float>(right_camera_info_.d[0]);
        calib_.rightDistortion.k2 = static_cast<float>(right_camera_info_.d[1]);
        calib_.rightDistortion.p1 = static_cast<float>(right_camera_info_.d[2]);
        calib_.rightDistortion.p2 = static_cast<float>(right_camera_info_.d[3]);
        calib_.rightDistortion.k3 = static_cast<float>(right_camera_info_.d[4]);

        // 2. Extract Rotation (R) 
        // In ROS, CameraInfo contains the rectification transforms. 
        // To get the relative rotation between cameras: R_rel = R_right^T * R_left
        cv::Mat R_l(3, 3, CV_64F, left_camera_info_.r.data());
        cv::Mat R_r(3, 3, CV_64F, right_camera_info_.r.data());
        cv::Mat R_relative = R_r.t() * R_l; 

        for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            // Accessing OpenCV row i, column j
            calib_.rotation[i][j] = R_relative.at<double>(i, j);
        }
    }

        // 3. Extract Translation (T)
        // In ROS, the baseline is Tx = P_right[3] / P_right[0]
        double fx = right_camera_info_.p[0];
        double Tx = right_camera_info_.p[3] / fx; // This is -Baseline * fx

        calib_.translation[0] = static_cast<float>(Tx); 
        calib_.translation[1] = 0.0f; // Standard ROS stereo assumes alignment in Y and Z
        calib_.translation[2] = 0.0f;
        calib_.imageWidth = left_camera_info_.width;
        calib_.imageHeight = left_camera_info_.height;
        return true;
    } catch (...) {
        return false;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RetinifyDisparityNode)