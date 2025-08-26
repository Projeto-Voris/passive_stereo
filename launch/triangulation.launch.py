from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution

def generate_launch_description():
    
    return LaunchDescription([
        LaunchArg('namespace', default_value=['Passive'], description='Namespace of topics'),
        LaunchArg('left_image', default_value=['left/image_raw'], description='stereo left image'),
        LaunchArg('right_image', default_value=['right/image_raw'], description='stereo right image'),
        LaunchArg('left_info', default_value=['left/camera_info'], description='left camera info'),
        LaunchArg('right_info', default_value=['right/camera_info'], description='right camera info'),

        Node(
            package='passive_stereo',
            namespace=LaunchConfig('namespace'),
            executable='triangulation',
            name='triangulation',
            arguments=[
                PathJoinSubstitution([
                    TextSubstitution(text='/'),
                    LaunchConfig('namespace'),
                    LaunchConfig('left_info')
                ]),
            ],
            parameters=[{'frame_id': 'Passive/left_camera_link'},
                        {'sampling_factor': 0.5},# downsample the image for faster processing in PCL (%)],
                        {'crop_factor': 0.8}], # crop the image from center (%)
            remappings=[
                ('disparity/image', 'disparity/image'),
                ('pointcloud', 'disparity/pointcloud'),
                ('left/rect_image', 'left/rect_image')
            ]
        )
    ])


