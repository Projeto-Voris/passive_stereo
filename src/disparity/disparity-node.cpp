#include "disparity.hpp"


#include<string>

using std::placeholders::_1;

int numDisparities = 12 * 16;
int blockSize = 75;
int preFilterType = 0;
int preFilterSize = 25;
int preFilterCap = 15;
int minDisparity = 1;
int textureThreshold = 61;
int uniquenessRatio = 0;
int speckleRange = 50;
int speckleWindowSize = 38;
int disp12MaxDiff = 20;
float lambda = 8000;
float sigma = 1.5;

cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(stereo);
cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(stereo);


DisparityNode::DisparityNode(sensor_msgs::msg::CameraInfo infoL, sensor_msgs::msg::CameraInfo infoR): Node(
    "disparity_node") {
    std::string left_image_topic = "/left/image_raw";
    std::string right_image_topic = "/right/image_raw";
    std::string params_topic = "/params";

    // Declare the ROS 2 parameter for the YAML file path
    this->declare_parameter<std::string>("stereo_params_file", ""); // default is an empty string
    this->declare_parameter<bool>("publish_rectified", false); // default is false
    this->declare_parameter<bool>("wls_filter", false); // default is false


    // Retrieve the YAML file path from the ROS 2 parameter
    std::string stereo_params_file;
    this->get_parameter("publish_rectified", publish_rectified);
    this->get_parameter("wls_filter", wls_filter_enabled);
    this->get_parameter("stereo_params_file", stereo_params_file);
    // If the parameter is valid, load the stereo parameters from the YAML file
    if (!stereo_params_file.empty()) {
        DisparityNode::loadYamlfile(stereo_params_file);
    } else {
        RCLCPP_ERROR(this->get_logger(), "No stereo parameters file provided.");
    }


    left_camera_info = infoL;
    right_camera_info = infoR;

    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, left_image_topic);
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, right_image_topic);


    CalculateRectificationRemaps();

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(
        approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(std::bind(&DisparityNode::GrabStereo, this, std::placeholders::_1,
                                                std::placeholders::_2));

    params_sub = create_subscription<std_msgs::msg::Int16MultiArray>(params_topic, 10,
                                                                     std::bind(&DisparityNode::UpdateParameters, this,
                                                                               _1));
    float Tx = right_camera_info.p[3];
    float fx_ = right_camera_info.p[0];
    baseline = Tx/(-fx_);


    focal_length = left_camera_info.k[0];

    disparity_publisher = this->create_publisher<stereo_msgs::msg::DisparityImage>("disparity_image", 10);
    if (publish_rectified) {
        rect_left_publisher = this->create_publisher<sensor_msgs::msg::Image>("left/rect_image", 10);
        rect_right_publisher = this->create_publisher<sensor_msgs::msg::Image>("right/rect_image", 10);
    }
}

void DisparityNode::GrabStereo(const ImageMsg::ConstSharedPtr msgLeft, const ImageMsg::ConstSharedPtr msgRight) {
    // Get messages of topic and convert to
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft, msgLeft->encoding);
        cv_ptrRight = cv_bridge::toCvShare(msgRight, msgLeft->encoding);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    auto imgmsg = sensor_msgs::msg::Image();
    auto dispmsg = stereo_msgs::msg::DisparityImage();
    cv::Mat imgL, imgR;
    if(msgLeft->encoding == msgRight->encoding && msgLeft->encoding == "mono8") {

        imgL = cv_ptrLeft->image;
        imgR = cv_ptrRight->image;
    }
    else {
        cv::cvtColor(cv_ptrLeft->image, imgL,  cv::COLOR_BGR2GRAY);
        cv::cvtColor(cv_ptrRight->image, imgR,  cv::COLOR_BGR2GRAY);
    }
    cv::Mat disp, disparity, raw_right_disparity_map, right_disparity;
    
    RectifyImages(imgL, imgR);
    // cv::Mat rectImgL = cv_ptrLeft->image;
    // cv::Mat rectImgR = cv_ptrRight->image;
    // RCLCPP_INFO(this->get_logger(), "Rect L resolution: width=%d, height=%d", rectImgL.cols, rectImgL.rows);
    // RCLCPP_INFO(this->get_logger(), "Rect R resolution: width=%d, height=%d", rectImgR.cols, rectImgR.rows);
    
    stereo->compute(rectImgL, rectImgR, disp);
    disp.convertTo(disparity,CV_32FC1);
    
    if(wls_filter_enabled){
        cv::Mat filtered_disparity_map, filtered_disparity_map_16u;
        right_matcher->compute(rectImgR, rectImgL, raw_right_disparity_map);


        wls_filter->filter(disp, rectImgL, filtered_disparity_map, raw_right_disparity_map);

        // raw_right_disparity_map.convertTo(right_disparity, CV_32FC1, 1);
        filtered_disparity_map.convertTo(filtered_disparity_map_16u, CV_32FC1);

        cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", filtered_disparity_map_16u).toImageMsg(imgmsg);
    }
    else{   
        cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", disparity).toImageMsg(imgmsg);
    }

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
}

void DisparityNode::UpdateParameters(const std_msgs::msg::Int16MultiArray::ConstSharedPtr params_message) {
    RCLCPP_INFO(this->get_logger(), "Received: %d", params_message->data[0]);
    stereo->setPreFilterCap(params_message->data[0]); //1 - 63
    stereo->setPreFilterSize(params_message->data[1]); // 5 - 255 impar
    stereo->setPreFilterType(params_message->data[2]);

    stereo->setTextureThreshold(params_message->data[3]);
    stereo->setUniquenessRatio(params_message->data[4]);

    stereo->setNumDisparities(params_message->data[5]); // positivo %16 == 0
    stereo->setBlockSize(params_message->data[6]); // 5- 255 impar
    stereo->setSpeckleRange(params_message->data[7]);

    stereo->setSpeckleWindowSize(params_message->data[8]);
    stereo->setDisp12MaxDiff(params_message->data[9]);
    stereo->setMinDisparity(params_message->data[10]);

    wls_filter->setLambda(params_message->data[11]);
    wls_filter->setSigmaColor(params_message->data[12]);

    minDisparity = params_message->data[10];
    numDisparities = params_message->data[5];
}

void DisparityNode::RectifyImages(cv::Mat imgL, cv::Mat imgR) {
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

void DisparityNode::CalculateRectificationRemaps() {
    cv::Mat intrinsics_left;
    cv::Mat dist_coeffs_left;

    cv::Mat intrinsics_right;
    cv::Mat dist_coeffs_right;

    cv::Mat R1, R2, P1, P2, Q;

    cv::Size siz;

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

    siz.width = left_camera_info.width;
    siz.height = left_camera_info.height;

    baseline = right_camera_info.p[3] / right_camera_info.p[0];

    RCLCPP_INFO(this->get_logger(), "Baseline: %f", baseline);

    cv::initUndistortRectifyMap(intrinsics_left, dist_coeffs_left, R1, P1, siz, CV_32FC1, left_map1, left_map2);
    cv::initUndistortRectifyMap(intrinsics_right, dist_coeffs_right, R2, P2, siz, CV_32FC1, right_map1, right_map2);
}

void DisparityNode::loadYamlfile(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open the YAML file: %s", filename.c_str());
        return;
    }

    fs["numDisparities"] >> numDisparities;
    fs["blockSize"] >> blockSize;
    fs["preFilterType"] >> preFilterType;
    fs["preFilterSize"] >> preFilterSize;
    fs["preFilterCap"] >> preFilterCap;
    fs["minDisparity"] >> minDisparity;
    fs["textureThreshold"] >> textureThreshold;
    fs["uniquenessRatio"] >> uniquenessRatio;
    fs["speckleRange"] >> speckleRange;
    fs["speckleWindowSize"] >> speckleWindowSize;
    fs["disp12MaxDiff"] >> disp12MaxDiff;
    fs["lambda"] >> lambda;
    fs["sigma"] >> sigma;

    // Ensure numDisparities is a multiple of 16
    if (numDisparities % 16 != 0) {
        RCLCPP_WARN(this->get_logger(), "numDisparities should be a multiple of 16, adjusting to nearest multiple.");
        numDisparities = (numDisparities / 16) * 16;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded parameters from YAML file:");
    RCLCPP_INFO(this->get_logger(), "numDisparities: %d", numDisparities);
    RCLCPP_INFO(this->get_logger(), "blockSize: %d", blockSize);
    RCLCPP_INFO(this->get_logger(), "preFilterType: %d", preFilterType);
    RCLCPP_INFO(this->get_logger(), "preFilterSize: %d", preFilterSize);
    RCLCPP_INFO(this->get_logger(), "preFilterCap: %d", preFilterCap);
    RCLCPP_INFO(this->get_logger(), "minDisparity: %d", minDisparity);
    RCLCPP_INFO(this->get_logger(), "textureThreshold: %d", textureThreshold);
    RCLCPP_INFO(this->get_logger(), "uniquenessRatio: %d", uniquenessRatio);
    RCLCPP_INFO(this->get_logger(), "speckleRange: %d", speckleRange);
    RCLCPP_INFO(this->get_logger(), "speckleWindowSize: %d", speckleWindowSize);
    RCLCPP_INFO(this->get_logger(), "disp12MaxDiff: %d", disp12MaxDiff);
    RCLCPP_INFO(this->get_logger(), "lambda: %f", lambda);
    RCLCPP_INFO(this->get_logger(), "sigma: %f", sigma);

    // Set stereo parameters based on loaded values
    stereo->setNumDisparities(numDisparities);
    stereo->setBlockSize(blockSize);
    stereo->setPreFilterCap(preFilterCap);
    stereo->setPreFilterSize(preFilterSize);
    stereo->setPreFilterType(preFilterType);
    stereo->setTextureThreshold(textureThreshold);
    stereo->setUniquenessRatio(uniquenessRatio);
    stereo->setSpeckleRange(speckleRange);
    stereo->setSpeckleWindowSize(speckleWindowSize);
    stereo->setDisp12MaxDiff(disp12MaxDiff);
    stereo->setMinDisparity(minDisparity);

    if (wls_filter_enabled) {
        wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);
    }

}