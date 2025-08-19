#include "triangulation.hpp"

using std::placeholders::_1;

TriangulationNode::TriangulationNode(sensor_msgs::msg::CameraInfo camera_info): Node("triangulation_rgb") {
    
    this->declare_parameter("frame_id", "left_camera_link");
    this-declare_parameter<int>("sampling_factor", 4);

    this->get_parameter("sampling_factor", sampling_factor);
    
    disparity_sub = std::make_shared<message_filters::Subscriber<stereo_msgs::msg::DisparityImage> >(this, "disparity_image");
    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, "/left/image_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(50), *disparity_sub, *left_sub);

    syncApproximate->registerCallback(std::bind(&TriangulationNode::GrabImages, this, std::placeholders::_1,std::placeholders::_2));

    principal_x_ = camera_info.p[2];
    principal_y_ = camera_info.p[6];
    fx_ = camera_info.p[0];
    fy_ = camera_info.p[5];
    RCLCPP_INFO(this->get_logger(), "fx: %f, fy: %f, cx: %f, cy: %f", fx_, fy_, principal_x_, principal_y_);

    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
}

void TriangulationNode::GrabImages(const ImageMsg::ConstSharedPtr disp_msg,
                                   const sensor_msgs::msg::Image::ConstSharedPtr left_msg) {
    // PointCloud2 message
    sensor_msgs::msg::PointCloud2 pointcloudmsg;
    baseline_ = disp_msg->t;
    fx_ = disp_msg->f;

    // Convert disparity and left images to OpenCV format
    cv::Mat image;
    try {
        cv_ptr_disp = cv_bridge::toCvCopy(disp_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
        if (left_msg->encoding == sensor_msgs::image_encodings::BGR8){
            cv_ptr_left = cv_bridge::toCvShare(left_msg, sensor_msgs::image_encodings::BGR8);
            image = cv_ptr_left->image;
        }
        if (left_msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB8){
            cv_ptr_left = cv_bridge::toCvShare(left_msg, sensor_msgs::image_encodings::BAYER_RGGB8);
            cv::cvtColor(cv_ptr_left->image, image, cv::COLOR_BayerRG2BGR);

        }
        if (left_msg->encoding == sensor_msgs::image_encodings::MONO8){
            cv_ptr_left = cv_bridge::toCvShare(left_msg, sensor_msgs::image_encodings::MONO8);
            image = cv_ptr_left->image;

        }
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    int width = cv_ptr_disp->image.cols;
    int height = cv_ptr_disp->image.rows;

    // Set PointCloud2 header
    pointcloudmsg.header.stamp = disp_msg->header.stamp;
    pointcloudmsg.header.frame_id =  this->get_parameter("frame_id").as_string();
    // pointcloudmsg.width = 1;  // Points per row
    // pointcloudmsg.height = height*width; // Number of rows
    pointcloudmsg.height = height / sampling_factor;
    pointcloudmsg.width = width / sampling_factor;
    pointcloudmsg.is_dense = false; // Allow NaN points
    pointcloudmsg.is_bigendian = false; // Little-endian (default for most systems)

    // Add fields for x, y, z, and optionally rgb based on encoding
    sensor_msgs::PointCloud2Modifier modifier(pointcloudmsg);
    if (left_msg->encoding == sensor_msgs::image_encodings::BGR8 || left_msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB8) {
        // RCLCPP_INFO(this->get_logger(), "Pointcloud colored");
        modifier.setPointCloud2Fields(4,
                                      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        pointcloudmsg.point_step = 16;  // 4 fields * 4 bytes (x, y, z, rgb)
    } else {
        // RCLCPP_INFO(this->get_logger(), "Pointcloud NOT colored");
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

    // Vetor temporário para armazenar os pontos válidos
    std::vector<uint8_t> point_data;
    int valid_points = 0;

    for (int i = 0; i < height; i += sampling_factor) {
        for (int j = 0; j < width; j += sampling_factor) {
            float disparity = disparity_data[i * width + j];
            if (disparity >= 10) {
                float z = -baseline_ * fx_ / (disparity);
                float x = (j - principal_x_) * z / fx_;
                float y = (i - principal_y_) * z / fx_;

                // Adiciona x, y, z
                size_t offset = point_data.size();
                point_data.resize(offset + pointcloudmsg.point_step);
                memcpy(&point_data[offset], &x, sizeof(float));
                memcpy(&point_data[offset + 4], &y, sizeof(float));
                memcpy(&point_data[offset + 8], &z, sizeof(float));

                // Adiciona cor se necessário
                if (left_msg->encoding == sensor_msgs::image_encodings::BGR8 || 
                    left_msg->encoding == sensor_msgs::image_encodings::RGB8 ||
                    left_msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB8) {
                    cv::Vec3b bgr_v = image.at<cv::Vec3b>(i, j);
                    uint8_t r = bgr_v[2], g = bgr_v[1], b = bgr_v[0];
                    uint32_t bgr = ((uint32_t)b << 16 | (uint32_t)g << 8 | (uint32_t)r);
                    float bgr_float;
                    std::memcpy(&bgr_float, &bgr, sizeof(float));
                    memcpy(&point_data[offset + 12], &bgr_float, sizeof(float));
                }
                valid_points++;
            }
        }
    }

    // Ajusta o tamanho do PointCloud2 para o número de pontos válidos
    pointcloudmsg.width = valid_points;
    pointcloudmsg.height = 1;
    pointcloudmsg.row_step = pointcloudmsg.point_step * pointcloudmsg.width;
    pointcloudmsg.data = std::move(point_data);

    pointcloud_publisher_->publish(pointcloudmsg);
}

