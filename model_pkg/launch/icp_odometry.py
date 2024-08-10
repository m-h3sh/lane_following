import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import launch
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    remappings=[('scan','/scan'),('/rgb/image','/camera_forward/image_raw'),('rgb/camera_info','/camera_forward/camera_info'),('/depth/image','/camera_forward/depth/image_raw')]
    parameters_rtab = {
        'frame_id':'base_link',
        # 'use_sim_time':use_sim_time,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': True,
        'icp_odometry': True,
        'visual_odometry': True,
        'subscribe_odom': False,
        'approx_sync': True,
        'use_action_for_goal': True,
        # 'qos_scan':100,
        # 'qos_imu':qos,
        'Reg/Strategy':'1',
        'Reg/Force3DoF': 'true',
        'RGBD/NeighborLinkRefining': 'True',
        'Grid/RangeMin':'0.2',
        'Optimizer/GravitySigma':'0'
    }
    include_rtabmap_ros = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
            launch_arguments={
                "rtabmap_args": "--delete_db_on_start",
                # "rgb_topic": "/camera/color/image_raw",
                "depth_topic": "/camera_forward/depth/image_raw",
                # "camera_info_topic": "/camera/color/camera_info",
                "depth_camera_info_topic": "/camera_forward/depth/camera_info",
                "scan_cloud_topic": "/scan",
                "rgb_topic":"/camera_forward/image_raw",
                "camera_info_topic":"/camera_forward/camera_info",
                "frame_id": "base_link",
                "vo_frame_id": "odom",
                "subscribe_scan_cloud": "true",
                "approx_sync": "true",
                "approx_sync_max_interval": "0.01",
                "wait_imu_to_init": "false",
                "imu_topic": "/imu/data",
                "qos": "2",
                "rviz": "true",
                "publish_tf": "true",
                "use_sim_time": "true",
                "visual_odometry": "true",
                "icp_odometry": "true",
                "rtabmapviz": "false",
                "queue_size": "200",
                "publish_tf_map": "true",
                "output": "both"
            }.items()
        )
    
    rtab_slam_node = Node(
         package='rtabmap_slam', executable='rtabmap', output='screen',parameters=[parameters_rtab], remappings=remappings,
         arguments=['-d'],
      )
    rtab_localization_node = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        parameters=[parameters_rtab,],
        remappings=remappings)
    
    rtab_viz = Node(
        package = 'rtabmap_viz',executable='rtabmap_viz',output='screen',parameters=[parameters_rtab],remappings=remappings)
    

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(rtab_slam_node)
    launchDescriptionObject.add_action(rtab_localization_node)
    launchDescriptionObject.add_action(rtab_viz)
    # launchDescriptionObject.add_action(include_rtabmap_ros)
    # launchDescriptionObject.add_action(include_rtabmap_ros)
    return launchDescriptionObject