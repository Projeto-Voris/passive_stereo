from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # 1. Define Launch Configurations
    ns = LaunchConfiguration('namespace')
    left_img_topic = LaunchConfiguration('left_image')
    right_img_topic = LaunchConfiguration('right_image')
    left_info_topic = LaunchConfiguration('left_info')
    right_info_topic = LaunchConfiguration('right_info')

    # 2. Define the Composable Node (Plugin)
    # We assign it to a variable first, then put it in the container
    retinify_node = ComposableNode(
        package='passive_stereo',
        plugin='RetinifyDisparityNode', # Ensure this matches your registration macro
        name='retinify_node',
        namespace=ns,
        parameters=[{
            'publish_rectified': True,
            'debug_image': True
        }],
        remappings=[
            ('left/image_raw', left_img_topic),
            ('right/image_raw', right_img_topic),
            ('left/camera_info', left_info_topic),
            ('right/camera_info', right_info_topic),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 3. Create the Container
    container = ComposableNodeContainer(
        name='retinify_container',
        namespace=ns,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[retinify_node],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='Passive', description='Namespace of topics'),
        DeclareLaunchArgument('left_image', default_value='left/image_raw', description='stereo left image'),
        DeclareLaunchArgument('right_image', default_value='right/image_raw', description='stereo right image'),
        DeclareLaunchArgument('left_info', default_value='left/camera_info', description='left camera info'),
        DeclareLaunchArgument('right_info', default_value='right/camera_info', description='right camera info'),
        container
    ])

