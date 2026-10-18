from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(get_package_share_directory("crawler_bringup"), "config", "crawler.yaml")
    return LaunchDescription([
        # Crawler Sensors
        # from crawler_sensors package and ros
        Node(
            package="crawler_sensors",
            executable="ina226_node",
            name="crawler_ina226_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="crawler_sensors",
            executable="icm20948_node",
            name="crawler_icm20948_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="crawler_sensors",
            executable="tmag_odom_pair_node",
            name="crawler_tmag_odom_pair_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="hls_lfcd_lds_driver",
            executable="hlds_laser_publisher",
            name="crawler_hlds_laser_publisher",
            output="screen",
            parameters=[config],
        ),

        # Crawler Safety
        # from crawler safety package
        Node(
            package="crawler_safety",
            executable="current_guard_node",
            name="crawler_current_guard_node",
            output="screen",
        ),
        Node(
            package='crawler_safety',
            executable='cliff_costmap_node',
            name='crawler_cliff_costmap_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='crawler_safety',
            executable='cliff_keepout_node',
            name='crawler_cliff_keepout_node',
            output='screen',
            parameters=[config],
        ),

        # Crawler Control
        # from crawler_control package
        Node(
            package="crawler_control",
            executable="odom",
            name="crawler_odom_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="crawler_control",
            executable="velocity_target",
            name="crawler_velocity_target",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="crawler_control",
            executable="motor_driver",
            name="crawler_motor_driver",
            output="screen",
        ),
        Node(
            package="crawler_control",
            executable="teleop",
            name="crawler_teleop",
            output="screen",
        ),
        
        # Crawler Static Transforms
        # Crawler Static Transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link_tf',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.036',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'base_link',
            ]
        ),

        # Measured IMU position relative to base_footprint: (0, -0.062, +0.053) m.
        # base_link is already +0.036 m above base_footprint, so z = +0.017 m here.
        # IMPORTANT: zero rotation is provisional pending IMU board axis verification.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link_tf',
            arguments=[
                '--x', '0.0', '--y', '-0.062', '--z', '0.017',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_scan_tf',
            arguments=[
                '--x', '-0.002854',
                '--y', '0.0',
                '--z', '0.115',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '3.1415926535',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_scan',
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_cliff_sensor_04_tf',
            arguments=[
                '--x', '0.226901',
                '--y', '0.152500',
                '--z', '0.061000',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'cliff_sensor_04',
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_cliff_sensor_05_tf',
            arguments=[
                '--x', '0.226901',
                '--y', '-0.152500',
                '--z', '0.061000',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'cliff_sensor_05',
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_cliff_sensor_06_tf',
            arguments=[
                '--x', '-0.205823',
                '--y', '-0.152500',
                '--z', '0.061000',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'cliff_sensor_06',
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_cliff_sensor_07_tf',
            arguments=[
                '--x', '-0.205823',
                '--y', '0.152500',
                '--z', '0.061000',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'cliff_sensor_07',
            ]
        ),
    ])