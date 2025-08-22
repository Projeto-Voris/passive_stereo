#include "retinify_disp_ipc.hpp"
#include <cv_bridge/cv_bridge.h>


using std::placeholders::_1;
using std::placeholders::_2;

RetinifyDisparityNode::RetinifyDisparityNode(sensor_msgs::msg::CameraInfo infoL, sensor_msgs::msg::CameraInfo infoR)
: Node("retinify_disparity_ipc", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    // Declare the ROS 2 parameter for the YAML file path
    this->declare_parameter<bool>("publish_rectified", false); // default is false
    this->declare_parameter<bool>("debug_image", false);
        
    // Retrieve the YAML file path from the ROS 2 parameter
    std::string stereo_params_file;
    this->get_parameter("publish_rectified", publish_rectified);
    this->get_parameter("debug_image", debug_image);
    RCLCPP_INFO(this->get_logger(), "Initalize process");
    // INITIALIZE THE PIPELINE
    pipeline.Initialize();
    
    left_camera_info = infoL;
    right_camera_info = infoR; 
    CalculateRectificationRemaps();
    // Subs das imagens estéreo (mantive message_filters, como no seu código original)
    left_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "left/image_raw");
    right_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "right/image_raw");

    sync_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
                approximate_sync_policy(10), *left_sub_, *right_sub_);
    sync_->registerCallback(std::bind(&RetinifyDisparityNode::grabStereo, this, _1, _2));

    // Publisher da disparidade usando IPC (publica unique_ptr)
    pub_disp_ = this->create_publisher<stereo_msgs::msg::DisparityImage>("disparity/image", 10);
        if (publish_rectified) {
            rect_left_publisher = this->create_publisher<sensor_msgs::msg::Image>("left/rect_image", 10);
            rect_right_publisher = this->create_publisher<sensor_msgs::msg::Image>("right/rect_image", 10);
        }
        if (debug_image){
            RCLCPP_INFO(this->get_logger(), "Publishing debug disp image");
            debug_disp_publisher = this->create_publisher<sensor_msgs::msg::Image>("disparity/debug/image", 10);
        }
}

void RetinifyDisparityNode::grabStereo(const ImageMsg::ConstSharedPtr msgLeft, const ImageMsg::ConstSharedPtr msgRight)
{
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try {
        cv_ptrLeft  = cv_bridge::toCvShare(msgLeft, msgLeft->encoding);
        cv_ptrRight = cv_bridge::toCvShare(msgRight, msgRight->encoding);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
        return;
    }

    RectifyImages(cv_ptrLeft->image.clone(), cv_ptrRight->image.clone(), msgLeft);
    cv::Mat disparity;

    pipeline.Run(rectImgL, rectImgR, disparity);
    // Cria DisparityImage usando unique_ptr (zero-copy)
    if (debug_image){
        cv::Mat img_debug;
        auto debug_msg = sensor_msgs::msg::Image();
        img_debug = retinify::tools::ColorizeDisparity(disparity, 256);
        cv::resize(img_debug, img_debug,cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
        cv_bridge::CvImage(std_msgs::msg::Header(), "8UC3", img_debug).toImageMsg(debug_msg);
        debug_disp_publisher->publish(debug_msg);

    }
    auto disp_msg = std::make_unique<stereo_msgs::msg::DisparityImage>();
    
    // Converte disparity em sensor_msgs::Image
    disp_msg->header = msgLeft->header;
    cv_bridge::CvImage cv_disp(disp_msg->header, sensor_msgs::image_encodings::TYPE_32FC1, disparity);
    
    disp_msg->image = *cv_disp.toImageMsg();
    disp_msg->f = focal_length_;   // já vem da sua calibração
    disp_msg->t = baseline_;       // idem
    disp_msg->min_disparity = 0;
    disp_msg->max_disparity = 256;
    disp_msg->delta_d = 1.0;

    // Publica movendo o unique_ptr -> intra-process comms garante zero-copy
    pub_disp_->publish(std::move(disp_msg));
}

void RetinifyDisparityNode::RectifyImages(cv::Mat imgL, cv::Mat imgR, const sensor_msgs::msg::Image::ConstSharedPtr msgLeft)
{
    cv::Mat imgL_color, imgR_color;

    // Se vier Bayer, converte para BGR antes de retificar
    if (msgLeft->encoding == sensor_msgs::image_encodings::BAYER_RGGB8) {
        cv::cvtColor(imgL, imgL_color, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(imgR, imgR_color, cv::COLOR_BayerRG2BGR);
    } else if (msgLeft->encoding == sensor_msgs::image_encodings::MONO8) {
        cv::cvtColor(imgL, imgL_color, cv::COLOR_GRAY2BGR);
        cv::cvtColor(imgR, imgR_color, cv::COLOR_GRAY2BGR);
    } else {
        imgL_color = imgL.clone();
        imgR_color = imgR.clone();
    }

    // Agora retifica as imagens já em BGR
    cv::remap(imgL_color, rectImgL, left_map1, left_map2, cv::INTER_LINEAR);
    cv::remap(imgR_color, rectImgR, right_map1, right_map2, cv::INTER_LINEAR);

    if (publish_rectified) {
        auto leftimgmsg = sensor_msgs::msg::Image();
        auto rightimgmsg = sensor_msgs::msg::Image();

        cv_bridge::CvImage(msgLeft->header,
                           sensor_msgs::image_encodings::RGB8,
                           rectImgL).toImageMsg(leftimgmsg);
        rect_left_publisher->publish(leftimgmsg);

        cv_bridge::CvImage(msgLeft->header,
                           sensor_msgs::image_encodings::RGB8,
                           rectImgR).toImageMsg(rightimgmsg);
        rect_right_publisher->publish(rightimgmsg);
    }
}

void RetinifyDisparityNode::CalculateRectificationRemaps() {
    cv::Mat intrinsics_left;
    cv::Mat dist_coeffs_left;

    cv::Mat intrinsics_right;
    cv::Mat dist_coeffs_right;

    cv::Mat R1, R2, P1, P2, Q;

    cv::Size size;

    focal_length_ = left_camera_info.p[0];

    cv::Mat(3, 3, CV_64F, left_camera_info.k.data()).copyTo(intrinsics_left);
    cv::Mat(3, 3, CV_64F, right_camera_info.k.data()).copyTo(intrinsics_right);

    // Ensure distortion coefficients are reshaped to a valid size
    cv::Mat(left_camera_info.d.size(), 1, CV_64F, left_camera_info.d.data()).copyTo(dist_coeffs_left);
    cv::Mat(right_camera_info.d.size(), 1, CV_64F, right_camera_info.d.data()).copyTo(dist_coeffs_right);

    cv::Mat(3, 3, CV_64F, left_camera_info.r.data()).copyTo(R1);
    cv::Mat(3, 3, CV_64F, right_camera_info.r.data()).copyTo(R2);
    cv::Mat(3, 4, CV_64F, left_camera_info.p.data()).copyTo(P1);
    cv::Mat(3, 4, CV_64F, right_camera_info.p.data()).copyTo(P2);

    size.width = left_camera_info.width;
    size.height = left_camera_info.height;

    baseline_ = right_camera_info.p[3] / right_camera_info.p[0];

    // RCLCPP_INFO(this->get_logger(), "Baseline: %f", baseline);

    cv::initUndistortRectifyMap(intrinsics_left, dist_coeffs_left, R1, P1, size, CV_32FC1, left_map1, left_map2);
    cv::initUndistortRectifyMap(intrinsics_right, dist_coeffs_right, R2, P2, size, CV_32FC1, right_map1, right_map2);
}