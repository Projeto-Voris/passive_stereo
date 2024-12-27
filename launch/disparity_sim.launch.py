from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        LaunchArg('left_image', default_value=['/stereo_left'], description='stereo left image'),
        LaunchArg('right_image', default_value=['/stereo_right'], description='stereo right image'),
        LaunchArg('left_info', default_value=['/stereo_left/camera_info'], description='left camera info'),
        LaunchArg('right_info', default_value=['/stereo_right/camera_info'], description='right camera info'),
        LaunchArg('stereo_params', default_value=['/sm2/disparity/stereo_params'],
                  description='Stereo params to config disp.'),
        LaunchArg('yaml_file', default_value=['stereo_rgb_heavy_sim.yaml'],
                  description='YAML file where is stereoBM config'),

        Node(
            package='passive_stereo',
            namespace='/sm2/disparity',
            executable='disparity',
            name='disparity',
            parameters=[{'stereo_params_file': PathJoinSubstitution(
                [FindPackageShare('passive_stereo'), 'cfg', LaunchConfig('yaml_file')])}],
            remappings=[
                ('/left/image_raw', LaunchConfig('left_image')),
                ('/right/image_raw', LaunchConfig('right_image')),
                ('/left/camera_info', LaunchConfig('left_info')),
                ('/right/camera_info', LaunchConfig('right_info')),
                ('/params', LaunchConfig('stereo_params'))
            ]
        )

    ])
