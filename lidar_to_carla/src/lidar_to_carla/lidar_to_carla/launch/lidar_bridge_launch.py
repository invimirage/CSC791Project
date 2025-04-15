from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'carla_host',
            default_value='127.0.0.1',
            description='CARLA server IP address'
        ),
        DeclareLaunchArgument(
            'carla_port',
            default_value='2000',
            description='CARLA server port'
        ),
        DeclareLaunchArgument(
            'lidar_debug',
            default_value='False',
            description='Use fake scan data from file'
        ),

        # Carla ROS Bridge
        # Node(
        #     package='carla_ros_bridge',
        #     executable='bridge',
        #     name='carla_ros_bridge',
        #     output='screen',
        #     parameters=[{
        #         'host': LaunchConfiguration('carla_host'),
        #         'port': LaunchConfiguration('carla_port'),
        #         'timeout': 200.0,
        #         'passive': False,
        #         'town': 'Town10HD_Opt',
        #     }]
        # ),

        # Lidar test publisher (replay from file)
        Node(
            package='lidar_to_carla',
            executable='lidar_obj_detect',
            name='lidar_obj_detect',
            output='screen',
            parameters=[{
                'lidar_debug': LaunchConfiguration('lidar_debug'),
                'fov_deg': 90.0,
                'car_size_threshold': [1.4, 2.5, 4.5],
                'max_distance': 1.0,
                'metal_intensity_threshold': 10.0,
                'world_scale_ratio': 40.0,
            }]
        )
    ])
