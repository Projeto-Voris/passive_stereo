from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    return LaunchDescription([
        LaunchArg('namespace', default_value=['SM2'], description='Namespace of topics'),
        LaunchArg('left_image', default_value=['left/image_raw'], description='stereo left image'),
        LaunchArg('right_image', default_value=['right/image_raw'], description='stereo right image'),
        LaunchArg('left_info', default_value=['left/camera_info'], description='left camera info'),
        LaunchArg('right_info', default_value=['right/camera_info'], description='right camera info'),
        LaunchArg('stereo_params', default_value=['stereo_params'],
                  description='Stereo params to config disp.'),
        LaunchArg('yaml_file_disp', default_value=['sm2_20EM4-C.yaml'],
                  description='YAML file where is stereoBM config'),

        Node(
            package='passive_stereo',
            namespace=LaunchConfig('namespace'),
            executable='disparity',
            name='disparity',
            arguments=[
                PathJoinSubstitution(['/', LaunchConfig('namespace'),LaunchConfig('left_info')]),
                PathJoinSubstitution(['/', LaunchConfig('namespace'),LaunchConfig('right_info')])
                ],
            parameters=[{'stereo_params_file': PathJoinSubstitution(
                [FindPackageShare('passive_stereo'), 'cfg', LaunchConfig('yaml_file_disp')]),
                'publish_rectified': False,}],
            remappings=[
                ('/left/image_raw', LaunchConfig('left_image')),
                ('/right/image_raw', LaunchConfig('right_image')),
                ('/params', LaunchConfig('stereo_params')),
                ('disparity_image', 'disparity/image')
            ] 
        ),
        Node(
            package='passive_stereo',
            namespace=LaunchConfig('namespace'),
            executable='triangulation_rgb',
            name='disparity_3D',
            arguments=[
                PathJoinSubstitution(['/', LaunchConfig('namespace'),LaunchConfig('left_info')])
            ],
            parameters=[{'frame_id': '/SM2/left_camera_link'}],
            remappings=[
                ('disparity_image', 'disparity/image'),
                ('pointcloud', 'disparity/pointcloud'),
                ('/left/image_raw', LaunchConfig('left_image'))
            ]
        )
    ])


