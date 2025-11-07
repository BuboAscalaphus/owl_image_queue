from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base_dir', default_value='/home/dev/bags'),
        DeclareLaunchArgument('acquisition_glob', default_value='"**/*.jpg"'),
        DeclareLaunchArgument('status_topic', default_value='/enabled'),
        DeclareLaunchArgument('process_when', default_value='false'),  # 'true' or 'false'
        DeclareLaunchArgument('velocity_topic', default_value='/velocity'),
        DeclareLaunchArgument('min_speed_mps', default_value='-1.0'),
        DeclareLaunchArgument('results_path', default_value=''),
        DeclareLaunchArgument('results_format', default_value='jsonl'),
        DeclareLaunchArgument('processor_name', default_value='noop'),
        DeclareLaunchArgument('max_batch', default_value='25'),
        DeclareLaunchArgument('image_processor_config_path', default_value=PathJoinSubstitution([FindPackageShare('owl_image_queue'), 'config','processing.params.yaml'])),

        Node(
            package='owl_image_queue',
            executable='enqueue_node',
            name='image_enqueue',
            parameters=[{
                'base_dir': LaunchConfiguration('base_dir'),
                'acquisition_glob': LaunchConfiguration('acquisition_glob'),
            }],
            output='screen'
        ),
        Node(
            package='owl_image_queue',
            executable='processor_node',
            name='image_processor',
            parameters=[
            # {
            #     'base_dir': LaunchConfiguration('base_dir'),
            #     'status_topic': LaunchConfiguration('status_topic'),
            #     'process_when': LaunchConfiguration('process_when'),
            #     'velocity_topic': LaunchConfiguration('velocity_topic'),
            #     'min_speed_mps': LaunchConfiguration('min_speed_mps'), 
            #     'results_path': LaunchConfiguration('results_path'),
            #     'results_format': LaunchConfiguration('results_format'),
            #     'processor_name': LaunchConfiguration('processor_name'),
            #     'max_batch': LaunchConfiguration('max_batch'),
            # },
                LaunchConfiguration('image_processor_config_path')
            ],
            output='screen'
        ),
    ])
