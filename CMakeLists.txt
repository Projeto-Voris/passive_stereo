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
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(retinify REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(disparity
        src/disparity/disparity.cpp
        src/disparity/disparity-node.cpp
)
ament_target_dependencies(disparity rclcpp sensor_msgs stereo_msgs cv_bridge message_filters)
target_link_libraries(disparity ${OpenCV_LIBS})

add_executable(triangulation
        src/triangulation/triangulation.cpp
        src/triangulation/triangulation-node.cpp
)
ament_target_dependencies(triangulation rclcpp sensor_msgs stereo_msgs cv_bridge message_filters)
target_link_libraries(triangulation ${OpenCV_LIBS})

add_executable(retinify_disp src/retinity_disp/retinify_disp.cpp src/retinity_disp/retinify_disp-node.cpp)
ament_target_dependencies(retinify_disp rclcpp sensor_msgs stereo_msgs cv_bridge message_filters)
target_link_libraries(retinify_disp retinify ${OpenCV_LIBS})

add_executable(passive_stereo src/passive_ipc/passive_ipc-node.cpp src/passive_ipc/retinify_disp_ipc.cpp src/passive_ipc/triangulation_ipc.cpp)
ament_target_dependencies(passive_stereo rclcpp sensor_msgs stereo_msgs cv_bridge message_filters)
target_link_libraries(passive_stereo retinify ${OpenCV_LIBS})

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

install(TARGETS disparity
                triangulation
                retinify_disp
                passive_stereo
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME})

ament_package()
