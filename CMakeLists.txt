cmake_minimum_required(VERSION 3.8)
project(passive_stereo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(retinify REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Define ROS dependencies
set(ROS_DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  stereo_msgs
  cv_bridge
  message_filters
)

include_directories(include)
link_directories(
  include
)

# add_executable(disp_node src/retinify_disp-node.cpp src/retinify_disp.cpp)
# ament_target_dependencies(disp_node rclcpp sensor_msgs stereo_msgs cv_bridge message_filters rclcpp_components)
# target_link_libraries(disp_node retinify ${OpenCV_LIBS})

add_library(retinify_disp SHARED src/retinify_disp.cpp)
ament_target_dependencies(retinify_disp rclcpp rclcpp_components sensor_msgs stereo_msgs cv_bridge message_filters OpenCV)
target_link_libraries(retinify_disp retinify ${OpenCV_LIBS})
rclcpp_components_register_node(retinify_disp 
  PLUGIN "RetinifyDisparityNode" 
  EXECUTABLE retinify_disp_node)

add_executable(disp_node src/retinify_disp-node.cpp src/retinify_disp.cpp)
ament_target_dependencies(disp_node rclcpp rclcpp_components sensor_msgs stereo_msgs cv_bridge message_filters OpenCV)
target_link_libraries(disp_node retinify ${OpenCV_LIBS})

add_library(disp_pointcloud SHARED src/triangulation.cpp)
ament_target_dependencies(disp_pointcloud rclcpp rclcpp_components sensor_msgs stereo_msgs cv_bridge message_filters tf2_ros tf2_geometry_msgs OpenCV)
target_link_libraries(disp_pointcloud ${OpenCV_LIBS})
rclcpp_components_register_node(disp_pointcloud 
  PLUGIN "TriangulationNode" 
  EXECUTABLE triangulation_node)

  add_executable(triangulation src/triangulation-node.cpp src/triangulation.cpp)
ament_target_dependencies(triangulation rclcpp rclcpp_components sensor_msgs stereo_msgs cv_bridge message_filters tf2_ros tf2_geometry_msgs OpenCV)
target_link_libraries(triangulation ${OpenCV_LIBS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
                triangulation
                disp_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
        retinify_disp
        disp_pointcloud
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)  

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME})

ament_package()
