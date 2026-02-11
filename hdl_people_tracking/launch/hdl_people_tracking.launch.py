from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='hdl_people_tracking_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hdl_people_tracking',
                plugin='hdl_people_tracking::HdlPeopleDetectionNode',
                name='hdl_people_detection_node',
                parameters=[{
                    'static_sensor': False,
                    'downsample_resolution': 0.1,
                    'backsub_resolution': 0.2,
                    'backsub_occupancy_thresh': 2,
                    'cluster_min_pts': 10,
                    'cluster_max_pts': 2048,
                    'cluster_min_size_x': 0.2,
                    'cluster_min_size_y': 0.2,
                    'cluster_min_size_z': 0.3,
                    'cluster_max_size_x': 1.0,
                    'cluster_max_size_y': 1.0,
                    'cluster_max_size_z': 2.0,
                    'enable_classification': True
                }],
                remappings=[
                    ('/odom', '/odom'),
                    ('/velodyne_points', '/velodyne_points')
                ]
            ),
            ComposableNode(
                package='hdl_people_tracking',
                plugin='hdl_people_tracking::HdlPeopleTrackingNode',
                name='hdl_people_tracking_node',
                parameters=[{
                    'remove_trace_thresh': 1.0,
                    'human_radius': 0.4
                }]
            )
        ],
        output='screen',
    )

    dummy_map_publisher = Node(
        package='hdl_people_tracking',
        executable='dummy_map_publisher.py',
        name='dummy_map_publisher',
        output='screen'
    )

    return LaunchDescription([container, dummy_map_publisher])
