#include "passive_stereo.hpp"


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


PassiveStereoNode::PassiveStereoNode(sensor_msgs::msg::CameraInfo infoL, sensor_msgs::msg::CameraInfo infoR): Node(
    "disparity_node") {
    std::string left_image_topic = "/left/image_raw";
    std::string right_image_topic = "/right/image_raw";
    std::string params_topic = "/params";

    // Declare the ROS 2 parameter for the YAML file path
    this->declare_parameter<std::string>("stereo_params_file", ""); // default is an empty string

    // Retrieve the YAML file path from the ROS 2 parameter
    std::string stereo_params_file;
    this->get_parameter("stereo_params_file", stereo_params_file);
    // If the parameter is valid, load the stereo parameters from the YAML file
    if (!stereo_params_file.empty()) {
        PassiveStereoNode::loadYamlfile(stereo_params_file);
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
    syncApproximate->registerCallback(std::bind(&PassiveStereoNode::GrabStereo, this, std::placeholders::_1,
                                                std::placeholders::_2));

    params_sub = create_subscription<std_msgs::msg::Int16MultiArray>(params_topic, 10,
                                                                     std::bind(&PassiveStereoNode::UpdateParameters, this,
                                                                               _1));
    float Tx = right_camera_info.p[3];
    float fx_ = right_camera_info.p[0];
    baseline = Tx/(-fx_);
    principal_x_ = left_camera_info.k[2];
    principal_y_ = left_camera_info.k[5];
    fx_ = left_camera_info.k[0];
    


    focal_length = left_camera_info.k[0];

    disparity_publisher = this->create_publisher<stereo_msgs::msg::DisparityImage>("disparity_image", 10);

    rect_left_publisher = this->create_publisher<sensor_msgs::msg::Image>("rect_left_image", 10);
    rect_right_publisher = this->create_publisher<sensor_msgs::msg::Image>("rect_right_image", 10);
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
}

void PassiveStereoNode::GrabStereo(const ImageMsg::ConstSharedPtr msgLeft, const ImageMsg::ConstSharedPtr msgRight) {
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
        // RCLCPP_ERROR(this->get_logger(), "mono8 encoding");
        // rectImgL = cv_ptrLeft->image;
        // rectImgR = cv_ptrRight->image; 
        imgL = cv_ptrLeft->image;
        imgR = cv_ptrRight->image;
    }
    else {
        cv::cvtColor(cv_ptrLeft->image, imgL,  cv::COLOR_BGR2GRAY);
        cv::cvtColor(cv_ptrRight->image, imgR,  cv::COLOR_BGR2GRAY);
    }
    cv::Mat disp, disparity, raw_right_disparity_map, right_disparity;
    cv::Mat filtered_disparity_map, filtered_disparity_map_16u;

    RectifyImages(imgL, imgR);
    // cv::Mat rectImgL = cv_ptrLeft->image;
    // cv::Mat rectImgR = cv_ptrRight->image;
    // RCLCPP_INFO(this->get_logger(), "Rect L resolution: width=%d, height=%d", rectImgL.cols, rectImgL.rows);
    // RCLCPP_INFO(this->get_logger(), "Rect R resolution: width=%d, height=%d", rectImgR.cols, rectImgR.rows);

    stereo->compute(rectImgL, rectImgR, disp);
    disp.convertTo(disparity,CV_32FC1);


    right_matcher->compute(rectImgR, rectImgL, raw_right_disparity_map);


    wls_filter->filter(disp, rectImgL, filtered_disparity_map, raw_right_disparity_map);

    // raw_right_disparity_map.convertTo(right_disparity, CV_32FC1, 1);
    filtered_disparity_map.convertTo(filtered_disparity_map_16u, CV_32FC1);

    cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", filtered_disparity_map_16u).toImageMsg(imgmsg);

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

    PublishPointCloud(dispmsg, *msgLeft);

}

void PassiveStereoNode::PublishPointCloud(stereo_msgs::msg::DisparityImage disp_msg, sensor_msgs::msg::Image left_msg){
     // PointCloud2 message
    sensor_msgs::msg::PointCloud2 pointcloudmsg;
    baseline_ = disp_msg.t;
    fx_ = disp_msg.f;

    // RCLCPP_INFO(this.get_logger(), "Processing disp at timestamp: %d", disp_msg.header.stamp.sec);
    // RCLCPP_INFO(this.get_logger(), "Processing left at timestamp: %d", left_msg.header.stamp.sec);

    // Convert disparity and left images to OpenCV format
    try {
        cv_ptr_disp = cv_bridge::toCvCopy(disp_msg.image, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr_left = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    int width = cv_ptr_disp->image.cols;
    int height = cv_ptr_disp->image.rows;
    int sampling_factor = 5; // Adjust this factor as needed

    // Set PointCloud2 header
    pointcloudmsg.header.stamp = now();// disp_msg.header.stamp;
    pointcloudmsg.header.frame_id =  this->get_parameter("frame_id").as_string();
    // pointcloudmsg.width = 1;  // Points per row
    // pointcloudmsg.height = height*width; // Number of rows
    pointcloudmsg.height = height / sampling_factor;
    pointcloudmsg.width = width / sampling_factor;
    pointcloudmsg.is_dense = false; // Allow NaN points
    pointcloudmsg.is_bigendian = false; // Little-endian (default for most systems)

    // Add fields for x, y, z, and optionally rgb based on encoding
    sensor_msgs::PointCloud2Modifier modifier(pointcloudmsg);
    if (left_msg.encoding == sensor_msgs::image_encodings::BGR8 || left_msg.encoding == sensor_msgs::image_encodings::RGB8) {
        modifier.setPointCloud2Fields(4,
                                      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        pointcloudmsg.point_step = 16;  // 4 fields * 4 bytes (x, y, z, rgb)
    } else {
        modifier.setPointCloud2Fields(3,
                                      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        pointcloudmsg.point_step = 12;  // 3 fields * 4 bytes (x, y, z)
    }

    // Set row step
    pointcloudmsg.row_step = pointcloudmsg.point_step * pointcloudmsg.width;

    // Resize the point cloud data array
    pointcloudmsg.data.resize(pointcloudmsg.row_step * pointcloudmsg.height);
    float* disparity_data = (float*)cv_ptr_disp->image.data;

    // Iterate through disparity map and generate point cloud
    for (int i = 0; i < height; i += sampling_factor) {
        for (int j = 0; j < width; j += sampling_factor) {
            float disparity = disparity_data[i * width + j];
            if (disparity >= 0) {
                // Compute 3D coordinates from disparity
                float z = 16*baseline_ * fx_ / (disparity);
                float x = (j - principal_x_) * z / fx_;
                float y = (i - principal_y_) * z / fy_;

                // Copy x, y, z to the point cloud message data
                int index = (i / sampling_factor * pointcloudmsg.width + j / sampling_factor) * pointcloudmsg.point_step;
                memcpy(&pointcloudmsg.data[index], &x, sizeof(float));
                memcpy(&pointcloudmsg.data[index + 4], &y, sizeof(float));
                memcpy(&pointcloudmsg.data[index + 8], &z, sizeof(float));

                // Optionally add RGB if encoding is BGR8 or RGB8
                if (left_msg.encoding == sensor_msgs::image_encodings::BGR8 || left_msg.encoding == sensor_msgs::image_encodings::RGB8) {
                    cv::Vec3b bgr = cv_ptr_left->image.at<cv::Vec3b>(i, j);
                    uint8_t r = bgr[2], g = bgr[1], b = bgr[0];
                    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                    memcpy(&pointcloudmsg.data[index + 12], &rgb, sizeof(uint32_t));
                }
            }
        }
    }

    // RCLCPP_INFO(this->get_logger(), "Publishing point cloud with width: %d, height: %d", pointcloudmsg.width, pointcloudmsg.height);
    pointcloud_publisher_->publish(pointcloudmsg);
}

void PassiveStereoNode::UpdateParameters(const std_msgs::msg::Int16MultiArray::ConstSharedPtr params_message) {
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

void PassiveStereoNode::RectifyImages(cv::Mat imgL, cv::Mat imgR) {
    cv::remap(imgL, rectImgL, left_map1, left_map2, cv::INTER_LANCZOS4);
    cv::remap(imgR, rectImgR, right_map1, right_map2, cv::INTER_LANCZOS4);

    auto leftimgmsg = sensor_msgs::msg::Image();
    auto rightimgmsg = sensor_msgs::msg::Image();

    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", rectImgL).toImageMsg(leftimgmsg);
    rect_left_publisher->publish(leftimgmsg);

    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", rectImgR).toImageMsg(rightimgmsg);
    rect_right_publisher->publish(rightimgmsg);
}

void PassiveStereoNode::CalculateRectificationRemaps() {
    cv::Mat intrinsics_left(3, 3, cv::DataType<double>::type);
    cv::Mat dist_coeffs_left(5, 1, cv::DataType<double>::type);

    cv::Mat intrinsics_right(3, 3, cv::DataType<double>::type);
    cv::Mat dist_coeffs_right(5, 1, cv::DataType<double>::type);

    cv::Mat rotation(3, 3, cv::DataType<double>::type);
    cv::Mat translation(3, 1, cv::DataType<double>::type);

    cv::Mat R1, R2, P1, P2, Q;

    cv::Size siz;

    focal_length = left_camera_info.k[0];

    intrinsics_left.at<double>(0, 0) = left_camera_info.k[0]; //fx
    intrinsics_left.at<double>(0, 2) = left_camera_info.k[2]; //cx
    intrinsics_left.at<double>(1, 1) = left_camera_info.k[4]; //fy
    intrinsics_left.at<double>(1, 2) = left_camera_info.k[5]; //cy
    intrinsics_left.at<double>(2, 2) = 1;

    dist_coeffs_left.at<double>(0) = left_camera_info.d[0]; //k1
    dist_coeffs_left.at<double>(1) = left_camera_info.d[1]; //k2
    dist_coeffs_left.at<double>(2) = left_camera_info.d[3]; //p1
    dist_coeffs_left.at<double>(3) = left_camera_info.d[4]; //p2
    dist_coeffs_left.at<double>(4) = left_camera_info.d[2]; //k3

    intrinsics_right.at<double>(0, 0) = right_camera_info.k[0];
    intrinsics_right.at<double>(0, 2) = right_camera_info.k[2];
    intrinsics_right.at<double>(1, 1) = right_camera_info.k[4];
    intrinsics_right.at<double>(1, 2) = right_camera_info.k[5];
    intrinsics_right.at<double>(2, 2) = 1;

    dist_coeffs_right.at<double>(0) = right_camera_info.d[0];
    dist_coeffs_right.at<double>(1) = right_camera_info.d[1];
    dist_coeffs_right.at<double>(2) = right_camera_info.d[3];
    dist_coeffs_right.at<double>(3) = right_camera_info.d[4];
    dist_coeffs_right.at<double>(4) = right_camera_info.d[2];

    siz.width = left_camera_info.width;
    siz.height = left_camera_info.height;

    rotation.at<double>(0, 0) = right_camera_info.p[0];
    rotation.at<double>(0, 1) = right_camera_info.p[1];
    rotation.at<double>(0, 2) = right_camera_info.p[2];
    rotation.at<double>(1, 0) = right_camera_info.p[4];
    rotation.at<double>(1, 1) = right_camera_info.p[5];
    rotation.at<double>(1, 2) = right_camera_info.p[6];
    rotation.at<double>(2, 0) = right_camera_info.p[8];
    rotation.at<double>(2, 1) = right_camera_info.p[9];
    rotation.at<double>(2, 2) = right_camera_info.p[10];

    baseline = right_camera_info.p[3];

    translation.at<double>(0) = right_camera_info.p[3];
    translation.at<double>(1) = right_camera_info.p[7];
    translation.at<double>(2) = right_camera_info.p[11];

    RCLCPP_INFO(this->get_logger(), "Baseline: %f", baseline);
    cv::stereoRectify(intrinsics_left, dist_coeffs_left, intrinsics_right, dist_coeffs_right, siz, rotation,
                      translation, R1, R2, P1, P2, Q);

    cv::initUndistortRectifyMap(intrinsics_left, dist_coeffs_left, R1, P1, siz, CV_32FC1, left_map1, left_map2);
    cv::initUndistortRectifyMap(intrinsics_right, dist_coeffs_right, R2, P2, siz, CV_32FC1, right_map1, right_map2);
}

void PassiveStereoNode::loadYamlfile(const std::string& filename) {
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

    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
}