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
    # Names 
    robotXacroName = 'igvc_robot'
    namePackage = 'model_pkg'
    # Relative Paths
    modelFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'model/green_world.world'
    configFileRelativePath = 'config/ekf.yaml'
    # Absolute Paths
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    pathRvizFile = os.path.join(get_package_share_directory(namePackage),'rviz/rviz_basic_settings.rviz')
    pathConfigFile = os.path.join(get_package_share_directory(namePackage), 'config/ekf.yaml')
    # Processing xacro
    robotDescription = xacro.process_file(pathModelFile).toxml()
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # qos = LaunchConfiguration('qos')
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
    
    remappings=[('scan','/scan'),('/camera/rgb/image_rect_color','/zed/zed_node/rgb/image_rect_color'),('rgb/camera_info','/zed/zed_node/rgb/camera_info'),('/depth/image','/zed/zed_node/depth/depth_registered')]
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))

    slam_launch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py'))

    # Declare the launch arguments  
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
	    name='urdf_model', 
	    default_value=pathModelFile, 
	    description='Absolute path to robot urdf file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
	    name='rviz_config_file',
	    default_value=pathRvizFile,
	    description='Full path to the RVIZ config file to use')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
	    name='gui',
	    default_value='True',
	    description='Flag to enable joint_state_publisher_gui')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
	    name='use_robot_state_pub',
	    default_value='True',
	    description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
	    name='use_rviz',
	    default_value='True',
	    description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
	    name='use_sim_time',
	    default_value='True',
	    description='Use simulation (Gazebo) clock if true')


    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={'world': pathWorldFile}.items()
    )


    spawnModelNode = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )

    params = {'robot_description': robotDescription, 'use_sim_time': True}

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])
    
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

#     robot_localization_node = Node(
#        package='robot_localization',
#        executable='ekf_node',
#        name='ekf_filter_node',
#        output='screen',
#        parameters=[pathConfigFile, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
# )
#     robot_localization_node = Node(
#        package='robot_localization',
#        executable='ekf_node',
#        name='ekf_filter_node',
#        output='screen',
#        parameters=[pathConfigFile, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
# )
    # include_rtabmap_ros = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
    #     launch_arguments={
    #         "rtabmap_args": "--delete_db_on_start",
    #         # "rgb_topic": "/camera/color/image_raw",
    #         "depth_topic": "/zed/zed_node/depth/depth_registered",
    #         # "camera_info_topic": "/camera/color/camera_info",
    #         "depth_camera_info_topic": "/zed/zed_node/depth/camera_info",
    #         "scan_cloud_topic": "/scan",
    #         "rgb_topic":"/zed/zed_node/rgb/image_rect_color",
    #         "camera_info_topic":"/zed/zed_node/rgb/camera_info",
    #         "frame_id": "base_link",
    #         "vo_frame_id": "odom",
    #         "subscribe_scan_cloud": "true",
    #         "approx_sync": "true",
    #         "approx_sync_max_interval": "0.01",
    #         "wait_imu_to_init": "false",
    #         "imu_topic": "/imu/data",
    #         "qos": "2",
    #         "queue_size": "1000",
    #         "rviz": "true",
    #         "publish_tf": "true",
    #         "use_sim_time": "false",
    #         "visual_odometry": "true",
    #         "icp_odometry": "true",
    #         "rtabmapviz": "false",
    #         "publish_tf_map": "true",
    #         "output": "both",
    #     }.items()
    # )
    
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
    
    launchDescriptionObject = LaunchDescription([
        # launch.actions.ExecuteProcess(cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'slam_params_file:=./src/model_pkg/config/mapper_params_online_async.yaml', 'use_sim_time:=true']),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose',"/home/mahesh/ros2_ws/src/model_pkg/model/world.world", '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', "use_sim_time:=true", "headless:=false"], output='screen'),
        launch.actions.ExecuteProcess(cmd=['ros2','run','tf2_ros','static_transform_publisher','0','0','0','0','0','0','map','odom']),
        # launch.actions.ExecuteProcess(cmd=['ros2','run','teleop_twist_keyboard','teleop_twist_keyboard','cmd_vel:=cmd_vel_joy'])
        ])
    launchDescriptionObject.add_action(declare_urdf_model_path_cmd)
    launchDescriptionObject.add_action(declare_rviz_config_file_cmd)
    launchDescriptionObject.add_action(declare_use_joint_state_publisher_cmd)
    launchDescriptionObject.add_action(declare_use_robot_state_pub_cmd)  
    launchDescriptionObject.add_action(declare_use_rviz_cmd) 
    launchDescriptionObject.add_action(declare_use_sim_time_cmd)
    # launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    # launchDescriptionObject.add_action(joint_state_publisher_node)
    # launchDescriptionObject.add_action(rtab_slam_node)
    # launchDescriptionObject.add_action(rtab_localization_node)
    # launchDescriptionObject.add_action(include_rtabmap_ros)
    # launchDescriptionObject.add_action(rtab_viz)
    # launchDescriptionObject.add_action(joint_state_publisher_gui_node)
    # launchDescriptionObject.add_action(robot_localization_node)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(static_transform_publisher)
    # launchDescriptionObject.add_action(include_rtabmap_ros)

    return launchDescriptionObject