#include "retinify_disp.hpp"


#include<string>

using std::placeholders::_1;


RetinityDisparityNode::RetinityDisparityNode(sensor_msgs::msg::CameraInfo infoL, sensor_msgs::msg::CameraInfo infoR): Node(
    "disparity_retinify_node") {

    // Declare the ROS 2 parameter for the YAML file path
    this->declare_parameter<bool>("publish_rectified", false); // default is false
        
    // Retrieve the YAML file path from the ROS 2 parameter
    std::string stereo_params_file;
    this->get_parameter("publish_rectified", publish_rectified);
    RCLCPP_INFO(this->get_logger(), "Initalize process");
    // INITIALIZE THE PIPELINE
    pipeline.Initialize();
    
    left_camera_info = infoL;
    right_camera_info = infoR;  

    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, "/Passive/left/image_raw");
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, "/Passive/right/image_raw");

    CalculateRectificationRemaps();

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(
        approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(std::bind(&RetinityDisparityNode::GrabStereo, this, std::placeholders::_1,
                                                std::placeholders::_2));
    
    disparity_publisher = this->create_publisher<stereo_msgs::msg::DisparityImage>("disparity_image", 10);

    if (publish_rectified) {
        rect_left_publisher = this->create_publisher<sensor_msgs::msg::Image>("left/rect_image", 10);
        rect_right_publisher = this->create_publisher<sensor_msgs::msg::Image>("right/rect_image", 10);
    }
}

void RetinityDisparityNode::GrabStereo(const ImageMsg::ConstSharedPtr msgLeft, const ImageMsg::ConstSharedPtr msgRight) {
    // RCLCPP_INFO(this->get_logger(),"GET IMAGES");
    // Get node parameters
    this->get_parameter("publish_rectified", publish_rectified);

    // Get messages of topic and convert to
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft, msgLeft->encoding);
        cv_ptrRight = cv_bridge::toCvShare(msgRight, msgLeft->encoding);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    auto dispmsg = stereo_msgs::msg::DisparityImage();
    auto imgmsg = sensor_msgs::msg::Image();


    cv::Mat imgL, imgR;
    if(msgLeft->encoding == msgRight->encoding && msgLeft->encoding == "mono8") {

        imgL = cv_ptrLeft->image;
        imgR = cv_ptrRight->image;
    }
    else {
        cv::cvtColor(cv_ptrLeft->image, imgL,  cv::COLOR_BayerRG2GRAY);
        cv::cvtColor(cv_ptrRight->image, imgR,  cv::COLOR_BayerRG2GRAY);
    }
    cv::Mat disp, disparity;

    RectifyImages(imgL, imgR);

    pipeline.Run(imgL, imgR, disparity);

    // // Print disparity size and encoding
    // RCLCPP_INFO(this->get_logger(), "Disparity size: %dx%d, type: %s",disparity.cols, disparity.rows,
    //             cv::typeToString(disparity.type()).c_str());
    // disp = retinify::tools::ColorizeDisparity(disparity, 256);

    // // Print disparity size and encoding
    // RCLCPP_INFO(this->get_logger(), "Disp size: %dx%d, type: %s",disp.cols, disp.rows,
    //             cv::typeToString(disp.type()).c_str());

    cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", disparity).toImageMsg(imgmsg);


    dispmsg.header = std_msgs::msg::Header();
    // dispmsg.header.stamp = this->get_clock()->now();
    dispmsg.header.stamp = msgLeft->header.stamp;
    dispmsg.header.frame_id = msgLeft->header.frame_id;
    dispmsg.image = imgmsg;
    dispmsg.min_disparity = 0;
    dispmsg.max_disparity = 1000;
    dispmsg.f = focal_length;
    dispmsg.t = baseline;
    dispmsg.delta_d = 1;
    disparity_publisher->publish(dispmsg);
    // RCLCPP_INFO(this->get_logger(), "Publish disp");
}

void RetinityDisparityNode::RectifyImages(cv::Mat imgL, cv::Mat imgR) {
    cv::remap(imgL, rectImgL, left_map1, left_map2, cv::INTER_LANCZOS4);
    cv::remap(imgR, rectImgR, right_map1, right_map2, cv::INTER_LANCZOS4);
      
    if (publish_rectified){
        auto leftimgmsg = sensor_msgs::msg::Image();
        auto rightimgmsg = sensor_msgs::msg::Image();

        cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", rectImgL).toImageMsg(leftimgmsg);
        rect_left_publisher->publish(leftimgmsg);

        cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", rectImgR).toImageMsg(rightimgmsg);
        rect_right_publisher->publish(rightimgmsg);
    }
}

void RetinityDisparityNode::CalculateRectificationRemaps() {
    cv::Mat intrinsics_left;
    cv::Mat dist_coeffs_left;

    cv::Mat intrinsics_right;
    cv::Mat dist_coeffs_right;

    cv::Mat R1, R2, P1, P2, Q;

    cv::Size size;

    focal_length = left_camera_info.k[0];

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

    baseline = right_camera_info.p[3] / right_camera_info.p[0];

    // RCLCPP_INFO(this->get_logger(), "Baseline: %f", baseline);

    cv::initUndistortRectifyMap(intrinsics_left, dist_coeffs_left, R1, P1, size, CV_32FC1, left_map1, left_map2);
    cv::initUndistortRectifyMap(intrinsics_right, dist_coeffs_right, R2, P2, size, CV_32FC1, right_map1, right_map2);
}