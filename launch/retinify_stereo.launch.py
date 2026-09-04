from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    left_img_topic = LaunchConfiguration('left_image')
    right_img_topic = LaunchConfiguration('right_image')
    left_info_topic = LaunchConfiguration('left_info')
    right_info_topic = LaunchConfiguration('right_info')
    depth_mode = LaunchConfiguration('depth_mode')
    publish_disparity = LaunchConfiguration('publish_disparity')
    publish_pointcloud = LaunchConfiguration('publish_pointcloud')
    publish_depth = LaunchConfiguration('publish_depth')
    publish_rectified = LaunchConfiguration('publish_rectified')
    debug_image = LaunchConfiguration('debug_image')
    sampling_factor = LaunchConfiguration('sampling_factor')
    crop_factor = LaunchConfiguration('crop_factor')
    min_disp = LaunchConfiguration('min_disp')
    max_dist = LaunchConfiguration('max_dist')
    frame_id = LaunchConfiguration('frame_id')
    parent_frame = LaunchConfiguration('parent_frame')
    calibration_file = LaunchConfiguration('calibration_file')

    retinify_stereo_node = ComposableNode(
        package='passive_stereo',
        plugin='passive_stereo::RetinifyStereoNode',
        name='retinify_stereo_node',
        namespace=ns,
        parameters=[{
            'depth_mode': depth_mode,
            'publish_disparity': publish_disparity,
            'publish_pointcloud': publish_pointcloud,
            'publish_depth': publish_depth,
            'publish_rectified': publish_rectified,
            'debug_image': debug_image,
            'sampling_factor': sampling_factor,
            'crop_factor': crop_factor,
            'min_disp': min_disp,
            'max_dist': max_dist,
            'frame_id': frame_id,
            'parent_frame': parent_frame,
            'calibration_file': calibration_file,
        }],
        remappings=[
            ('left/image_rect', left_img_topic),
            ('right/image_rect', right_img_topic),
            ('left/camera_info', left_info_topic),
            ('right/camera_info', right_info_topic),
            ('disparity/image', 'disparity/image'),
            ('disparity/pointcloud', 'disparity/pointcloud'),
            ('depth/image', 'depth/image'),
            ('disparity/debug/image', 'disparity/debug/image'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    container = ComposableNodeContainer(
        name='retinify_stereo_container',
        namespace=ns,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[retinify_stereo_node],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='Passive', description='Namespace of topics'),
        DeclareLaunchArgument('left_image', default_value='left/image_raw', description='Stereo left image topic'),
        DeclareLaunchArgument('right_image', default_value='right/image_raw', description='Stereo right image topic'),
        DeclareLaunchArgument('left_info', default_value='left/camera_info', description='Left camera info topic'),
        DeclareLaunchArgument('right_info', default_value='right/camera_info', description='Right camera info topic'),
        DeclareLaunchArgument('depth_mode', default_value='accurate', description='Retinify depth mode: fast, balanced, or accurate'),
        DeclareLaunchArgument('publish_disparity', default_value='false', description='Publish disparity image'),
        DeclareLaunchArgument('publish_pointcloud', default_value='true', description='Publish point cloud'),
        DeclareLaunchArgument('publish_depth', default_value='false', description='Publish depth image'),
        DeclareLaunchArgument('publish_rectified', default_value='false', description='Publish rectified images'),
        DeclareLaunchArgument('debug_image', default_value='true', description='Publish colorized debug disparity'),
        DeclareLaunchArgument('sampling_factor', default_value='0.5', description='Point cloud sampling factor (0.01 to 1.0)'),
        DeclareLaunchArgument('crop_factor', default_value='1.0', description='Crop factor (0.01 to 1.0)'),
        DeclareLaunchArgument('min_disp', default_value='1.0', description='Minimum disparity threshold'),
        DeclareLaunchArgument('max_dist', default_value='15.0', description='Max distance filter in meters'),
        DeclareLaunchArgument('frame_id', default_value='Passive/left_camera_link', description='Optical camera frame ID'),
        DeclareLaunchArgument('parent_frame', default_value='base_link', description='Optional parent frame for static TF transformation'),
        DeclareLaunchArgument('calibration_file', default_value='', description='Optional path to Retinify JSON calibration file'),
        container
    ])
