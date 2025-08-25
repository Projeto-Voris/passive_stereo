from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    return LaunchDescription([
        LaunchArg('namespace', default_value=['Passive'], description='Namespace of topics'),
        LaunchArg('left_image', default_value=['left/image_raw'], description='stereo left image'),
        LaunchArg('right_image', default_value=['right/image_raw'], description='stereo right image'),
        LaunchArg('left_info', default_value=['/Passive/left/camera_info'], description='left camera info'),
        LaunchArg('right_info', default_value=['/Passive/right/camera_info'], description='right camera info'),
        Node(
            package='passive_stereo',
            namespace=LaunchConfig('namespace'),
            executable='passive_stereo',
            name='pasive_stereo',
            arguments=[
                LaunchConfig('left_info'),
                LaunchConfig('right_info')
                ],
            parameters=[{'frame_id': 'Passive/left_camera_link'},
                        {'sampling_factor': 6},
                        {'publish_rectified': True},
                        {'debug_image': False},
                        {'crop_factor': 0.8}],
            remappings=[
                ('left/image_raw', LaunchConfig('left_image')),
                ('right/image_raw', LaunchConfig('right_image')),
                ('disparity_image', 'disparity/image'),
                ('pointcloud', 'disparity/pointcloud'),
            ],
        )
    ])


