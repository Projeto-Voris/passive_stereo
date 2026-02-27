#include "retinify_disp.hpp"
RetinifyDisparityNode::RetinifyDisparityNode(const rclcpp::NodeOptions & options)
: Node("retinify_disparity_ipc", options)
{
    this->declare_parameter<bool>("debug_image", true);
    this->declare_parameter<bool>("publish_disp", true);
    this->get_parameter("publish_disp", publish_disp_);
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
    left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "left/image_rect", subscribe_qos_profile.get_rmw_qos_profile());
    right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "right/image_rect", subscribe_qos_profile.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
                approximate_sync_policy(10), *left_sub_, *right_sub_);
    sync_->registerCallback(std::bind(&RetinifyDisparityNode::grabStereo, this, std::placeholders::_1, std::placeholders::_2));

    // 3. Publishers
    if(publish_disp_){
        pub_disp_ = this->create_publisher<stereo_msgs::msg::DisparityImage>("disparity/image", 10);
    }
    if (debug_image_){
        debug_disp_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("disparity/debug/image", debug_qos_profile);
    }
    RCLCPP_INFO(this->get_logger(), "Retinify Disparity Node initialized. Waiting for camera info and images...");
}

void RetinifyDisparityNode::grabcamInfoLeft(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (left_info_received_) return;
    left_camera_info_ = *msg;
    left_info_received_ = true;
    focal_length_ = -left_camera_info_.p[0]; // Assuming fx is at p[0]
    RCLCPP_INFO(this->get_logger(), "Left Camera Info received.");
}

void RetinifyDisparityNode::grabcamInfoRight(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (right_info_received_) return;
    right_camera_info_ = *msg;
    right_px_ = right_camera_info_.p[3]; // Assuming Tx is at p[3]
    right_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Right Camera Info received.");
}

void RetinifyDisparityNode::grabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr msgRight){
    // GUARD: Only proceed if we have both CameraInfos
    this->get_parameter("publish_disp", publish_disp_);
    this->get_parameter("debug_image", debug_image_);
    if (!left_info_received_ || !right_info_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for Camera Info...");
        return; 
    }

    // Initialize Maps and Pipeline ONCE
    if (!maps_initialized_) {
        auto status = pipeline.Initialize(static_cast<std::uint32_t>(left_camera_info_.width),
                                         static_cast<std::uint32_t>(left_camera_info_.height),
                                        retinify::PixelFormat::RGB8, 
                                        retinify::DepthMode::ACCURATE);
        RCLCPP_INFO(this->get_logger(), "Retinify pipeline initialized with image size: %dx%d", left_camera_info_.width, left_camera_info_.height);
        if (!status.IsOK()) {
            RCLCPP_ERROR(this->get_logger(), "Pipeline init failed");
            return;
        }
        maps_initialized_ = true;
    }
    RCLCPP_DEBUG(this->get_logger(), "Received synchronized stereo pair. Processing...");
    // Conversion: ensure images are in RGB8 format for the Retinify pipeline
    // The helper functions below will perform any necessary color conversion while
    // still using IPC-friendly shared pointers when possible.
    cv_bridge::CvImagePtr cv_ptrLeft, cv_ptrRight;
    try {
        // toCvCopy will convert to the requested encoding and return a new image
        // if necessary.  We request RGB8 explicitly.
        cv_ptrLeft  = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::RGB8);
        cv_ptrRight = cv_bridge::toCvCopy(msgRight, sensor_msgs::image_encodings::RGB8);
        RCLCPP_DEBUG(this->get_logger(), "Images converted to RGB8 format for Retinify.");
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion to RGB8 failed: %s", e.what());
        return;
    }

    // Retinify Execution
    if (pipeline.Execute(cv_ptrLeft->image.ptr<uint8_t>(), cv_ptrLeft->image.step[0], 
                         cv_ptrRight->image.ptr<uint8_t>(), cv_ptrRight->image.step[0]).IsOK()) {
        RCLCPP_DEBUG(this->get_logger(), "Retinify pipeline executed successfully.");
        cv::Mat disparity = cv::Mat::zeros(cv_ptrLeft->image.size(), CV_32FC1);
            // Retinify disparity retrieval
            if (pipeline.RetrieveDisparity(disparity.ptr<float>(), disparity.step[0]).IsOK()) {
                if(publish_disp_){
                    baseline_ = right_px_ / focal_length_;
                    RCLCPP_DEBUG(this->get_logger(), "Publishing disparity image...");
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
                    RCLCPP_DEBUG(this->get_logger(), "Colorizing disparity for debug image...");
                    cv::Mat disparityColored(disparity.size(), CV_8UC3);
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

RCLCPP_COMPONENTS_REGISTER_NODE(RetinifyDisparityNode)