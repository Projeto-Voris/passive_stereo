cmake_minimum_required(VERSION 3.8)
project(passive_stereo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall> $<$<COMPILE_LANGUAGE:CXX>:-Wextra>)
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

# Find OpenMP
find_package(OpenMP REQUIRED)

# Find and enable CUDA
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  set(CUDA_FOUND TRUE)
  if(NOT DEFINED CMAKE_CUDA_STANDARD)
    set(CMAKE_CUDA_STANDARD ${CMAKE_CXX_STANDARD})
  endif()
  
  # Configure CUDA architectures (87 = NVIDIA Jetson Orin Nano / Orin NX / AGX Orin)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
      # Target Jetson Orin (sm_87) and Xavier (sm_72) on ARM64
      set(CMAKE_CUDA_ARCHITECTURES 87 72)
    else()
      # x86_64 with Jetson cross/multi-target support
      set(CMAKE_CUDA_ARCHITECTURES 75 80 86 87 89)
    endif()
  endif()

  add_compile_definitions(HAVE_CUDA=1)
  message(STATUS "CUDA found: ${CMAKE_CUDA_COMPILER} - CUDA Archs: ${CMAKE_CUDA_ARCHITECTURES}")
else()
  set(CUDA_FOUND FALSE)
  message(WARNING "CUDA compiler not found - building with CPU-only mode")
endif()

include_directories(include)

# 1. Retinify Disparity Node
add_library(retinify_disp SHARED src/retinify_disp.cpp)
ament_target_dependencies(retinify_disp rclcpp rclcpp_components sensor_msgs stereo_msgs cv_bridge message_filters OpenCV)
target_link_libraries(retinify_disp retinify ${OpenCV_LIBS})
rclcpp_components_register_node(retinify_disp 
  PLUGIN "RetinifyDisparityNode" 
  EXECUTABLE retinify_disp_node)

add_executable(disp_node src/retinify_disp-node.cpp src/retinify_disp.cpp)
ament_target_dependencies(disp_node rclcpp rclcpp_components sensor_msgs stereo_msgs cv_bridge message_filters OpenCV)
target_link_libraries(disp_node retinify ${OpenCV_LIBS})

# 2. Triangulation Node (GPU + CPU)
if(CUDA_FOUND)
  set(TRIANGULATION_SRC
    src/triangulation.cpp
    src/triangulation_cuda.cu
  )
else()
  set(TRIANGULATION_SRC
    src/triangulation.cpp
  )
endif()

add_library(disp_pointcloud SHARED ${TRIANGULATION_SRC})
ament_target_dependencies(disp_pointcloud rclcpp rclcpp_components sensor_msgs stereo_msgs cv_bridge message_filters tf2_ros tf2_geometry_msgs OpenCV)
target_link_libraries(disp_pointcloud ${OpenCV_LIBS} OpenMP::OpenMP_CXX)
if(CUDA_FOUND)
  set_target_properties(disp_pointcloud PROPERTIES
    CUDA_SEPARABLE_COMPILATION OFF
  )
endif()
rclcpp_components_register_node(disp_pointcloud 
  PLUGIN "TriangulationNode" 
  EXECUTABLE triangulation_node)

add_executable(triangulation src/triangulation-node.cpp ${TRIANGULATION_SRC})
ament_target_dependencies(triangulation rclcpp rclcpp_components sensor_msgs stereo_msgs cv_bridge message_filters tf2_ros tf2_geometry_msgs OpenCV)
target_link_libraries(triangulation ${OpenCV_LIBS} OpenMP::OpenMP_CXX)
if(CUDA_FOUND)
  set_target_properties(triangulation PROPERTIES
    CUDA_SEPARABLE_COMPILATION OFF
  )
endif()

# 3. Stereo Rectify Node (Plugin)
add_library(stereo_rect SHARED src/stereo_rect.cpp)
ament_target_dependencies(stereo_rect rclcpp rclcpp_components sensor_msgs cv_bridge message_filters OpenCV)
target_link_libraries(stereo_rect ${OpenCV_LIBS})
rclcpp_components_register_node(stereo_rect
  PLUGIN "passive_stereo::StereoRectifyNode"
  EXECUTABLE stereo_rect_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
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
        stereo_rect
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)  

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME})

ament_package()
