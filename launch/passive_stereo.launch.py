from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    return LaunchDescription([
        LaunchArg('namespace', default_value=['SM2'], description='Namespace of topics'),
        LaunchArg('left_image', default_value=['left/image_raw'], description='stereo left image'),
        LaunchArg('right_image', default_value=['right/image_raw'], description='stereo right image'),
        LaunchArg('left_info', default_value=['left/camera_info'], description='left camera info'),
        LaunchArg('right_info', default_value=['right/camera_info'], description='right camera info'),
        Node(
            package='passive_stereo',
            namespace=LaunchConfig('namespace'),
            executable='passive_stereo',
            name='pasive_stereo',
            arguments=[
                PathJoinSubstitution([
                    TextSubstitution(text='/'),
                    LaunchConfig('namespace'),
                    LaunchConfig('left_info')
                ]),
                PathJoinSubstitution([
                    TextSubstitution(text='/'),
                    LaunchConfig('namespace'),
                    LaunchConfig('right_info')
                ]),
            ],
            parameters=[{'frame_id': 'SM2/left_camera_link'},
                        {'sampling_factor': 0.5}, # downsample the image for faster processing in PCL (%)
                        {'publish_rectified': True}, # publish rectified image
                        {'debug_image': True}, # publish disparity image for debug as image msg
                        {'crop_factor': 0.8}], # crop the image from center (%)
            remappings=[
                ('left/image_raw', LaunchConfig('left_image')),
                ('right/image_raw', LaunchConfig('right_image')),
                ('disparity_image', 'disparity/image'),
                ('pointcloud', 'disparity/pointcloud'),
            ],
        )
    ])


