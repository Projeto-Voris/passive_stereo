from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        LaunchArg('namespace', default_value=['passive_stereo'], description='Namespace of topics'),
        LaunchArg('left_image', default_value=['/stereo_left'], description='stereo left image topic'),
        LaunchArg('right_image', default_value=['/stereo_right'], description='stereo right image topic'),
        LaunchArg('left_info', default_value=['/stereo_left/camera_info'], description='left camera info topic'),
        LaunchArg('right_info', default_value=['/stereo_right/camera_info'], description='right camera info topic'),
        LaunchArg('disparity', default_value=['disparity_image'], description='disparity topic'),
        LaunchArg('pointcloud', default_value=['pointcloud'], description='disparity topic'),
        LaunchArg('frame_id', default_value=['left_camera_frame'], description='Pointcloud frame_id'),
        LaunchArg('stereo_params', default_value=['/disparity/stereo_params'], description='Stereo params to config disp. topic'),
        LaunchArg('yaml_file', default_value=['stereo_rgb_heavy_sim.yaml'], description='YAML file where is stereoBM config'),
        LaunchArg('frame_id', default_value=['left_camera_frame'], description='Pointcloud frame_id'),
        LaunchArg('debug_disparity', default_value='false', choices=['true','false'], description='See Disparity image using image_view'),


        Node(
            package='passive_stereo',
            namespace=LaunchConfig('namespace'),
            executable='passive_stereo',
            name='passive_stereo',
            arguments=[
                LaunchConfig('left_info'),
                LaunchConfig('right_info')
            ],

            parameters=[{'stereo_params_file': PathJoinSubstitution(
                [FindPackageShare('passive_stereo'), 'cfg', LaunchConfig('yaml_file')]), 'frame_id':LaunchConfig('frame_id')}],

            remappings=[
                ('/left/image_raw', LaunchConfig('left_image')),
                ('/right/image_raw', LaunchConfig('right_image')),
                ('/params', LaunchConfig('stereo_params'))
            ]
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'image_view', 'disparity_view', '--ros-args', '--remap', ['image:=','/',LaunchConfig('namespace'),'/',LaunchConfig('disparity')]],
            output='screen',
            condition = IfCondition(LaunchConfig('debug_disparity'))
        )
    ])
