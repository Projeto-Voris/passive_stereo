#include "triangulation_rgb.hpp"

using std::placeholders::_1;

TriangulationNode::TriangulationNode(sensor_msgs::msg::CameraInfo camera_info): Node("triangulation_rgb") {
    std::string disparity_image_topic = "disparity_image";
    std::string left_image_topic = "/left/image_raw";
    this->declare_parameter("frame_id", "left_camera_link");

    disparity_sub = std::make_shared<message_filters::Subscriber<stereo_msgs::msg::DisparityImage> >(this, disparity_image_topic);
    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, left_image_topic);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(50), *disparity_sub, *left_sub);

    syncApproximate->registerCallback(std::bind(&TriangulationNode::GrabImages, this, std::placeholders::_1,std::placeholders::_2));

    principal_x_ = camera_info.k[2];
    principal_y_ = camera_info.k[5];
    fx_ = camera_info.k[0];
    fy_ = camera_info.k[4];

    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
}

void TriangulationNode::GrabImages(const ImageMsg::ConstSharedPtr disp_msg,
                                   const sensor_msgs::msg::Image::ConstSharedPtr left_msg) {
    // PointCloud2 message
    sensor_msgs::msg::PointCloud2 pointcloudmsg;
    baseline_ = disp_msg->t;
    fx_ = disp_msg->f;

    // RCLCPP_INFO(this->get_logger(), "Processing disp at timestamp: %d", disp_msg->header.stamp.sec);
    // RCLCPP_INFO(this->get_logger(), "Processing left at timestamp: %d", left_msg->header.stamp.sec);

    // Convert disparity and left images to OpenCV format
    try {
        cv_ptr_disp = cv_bridge::toCvCopy(disp_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr_left = cv_bridge::toCvShare(left_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    int width = cv_ptr_disp->image.cols;
    int height = cv_ptr_disp->image.rows;
    int sampling_factor = 5; // Adjust this factor as needed

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
    if (left_msg->encoding == sensor_msgs::image_encodings::BGR8 || left_msg->encoding == sensor_msgs::image_encodings::RGB8) {
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
                if (left_msg->encoding == sensor_msgs::image_encodings::BGR8 || left_msg->encoding == sensor_msgs::image_encodings::RGB8) {
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

