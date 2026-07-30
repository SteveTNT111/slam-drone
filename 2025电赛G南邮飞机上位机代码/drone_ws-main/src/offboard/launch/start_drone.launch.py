import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('offboard')
    
    # 参数文件路径
    params_file = os.path.join(pkg_dir, 'config', 'offb_configs.yaml')
    
    # 定义仿真参数
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Whether to run in simulation mode'
    )



    return LaunchDescription([
        simulation_arg,
#############################################公用#############################################
        # 启动 Livox ROS Driver（仅在非仿真模式下）
        ExecuteProcess(
            cmd=['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py'],
            output='screen',
            name='livox_ros_driver',
            condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        ),

        # # 启动 Fast-LIO Mapping（仅在非仿真模式下）
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'fast_lio', 'mapping.launch.py'],
        #     output='screen',
        #     name='fast_lio_mapping',
        #     condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        # ),
        # 启动 Point-LIO Mapping（仅在非仿真模式下）
        ExecuteProcess(
            cmd=['ros2', 'launch', 'point_lio', 'point_lio.launch.py'],
            output='screen',
            name='Point_lio_mapping',
            condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        ),

        # # 启动 offboard_pkg 的 serial 节点（仅在非仿真模式下）
        # Node(
        #     package='gstation',
        #     executable='serial',
        #     output='screen',
        #     name='serial',
        #     condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        # ),

        # 设置权限 /dev/ttyACM0（仅在非仿真模式下）
        ExecuteProcess(
            cmd=['sudo', 'chmod', '666', '/dev/ttyACM0'],
            output='screen',
            name='set_tty_permissions',
            condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        ),

        # 设置权限 /dev/ttyUSB0（仅在非仿真模式下）
        ExecuteProcess(
            cmd=['sudo', 'chmod', '666', '/dev/ttyACM1'],
            output='screen',
            name='set_tty_permissions',
            condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        ),

        # 启动 PX4 与 MAVROS（仅在非仿真模式下）
        ExecuteProcess(
            cmd=['ros2', 'launch', 'mavros', 'px4.launch', 'fcu_url:=/dev/px4:921600'],
            # cmd=['ros2', 'launch', 'mavros', 'px4.launch', 'fcu_url:=/dev/ttyACM0:921600'],

            output='screen',
            name='mavros_px4',
            condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        ),


        # 启动 offboard_pkg 的 odom_to_pose_node 节点（仅在非仿真模式下）
        Node(
            package='offboard',
            executable='odom_to_pose_node',
            output='screen',
           name='odom_to_pose_node',
           condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        ),
#################################################杨##############################################
        
        # 启动 offb_node 节点（添加参数文件）
        Node(
            package='offboard',
            executable='offboard_node',
            name='offb_node',
            parameters=[params_file],  # 添加参数文件
            output='screen'
        ),




        # # 启动 Foxglove Bridge
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml'],
        #     output='log',
        #     name='foxglove_bridge'
        # ),

        # # 静态变换发布器：base_link → camera_link（D435）
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', 
        #          '0.15', '0', '-0.05', '0', '0', '0', 'base_link', 'camera_link'],
        #     output='log',
        #     name='static_tf_camera'
        # ),

        # # 静态变换发布器：base_link → body（LIO雷达）
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', 
        #          '0', '0', '0', '0', '0', '0', 'body', 'base_link'],
        #     output='log',
        #     name='static_tf_body'
        # ),
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', 
        #          '0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        #     output='log',
        #     name='static_tf_map'
        # ),


        # # 启动ego_planner
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'ego_planner', 'ego_planner_real.launch.py'],
        #     output='screen',
        #     name='ego_planner_launch',
        # ),
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'ego_planner', 'rviz.launch.py'],
        #     output='log',
        #     name='rviz',
        # ),

####################################################李#########################################
        # 启动地面站节点
        Node(
            package='gstation',
            executable='landscreen',
            name='landscreen',
            output='screen'
        ),

        Node(
            package='gstation',
            executable='gstation',
            name='gstation_node',
            parameters=[params_file],  # 添加参数文件
            output='screen',
            condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        ),
        # 启动地面站的串口节点
        Node(
            package='gstation',
            executable='serial',
            name='serial',
            output='screen',
            condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        ),
########################################姚#########################################
        Node(
            package='vision',          
            executable='detection_socket_receiver', 
            name='detection_receiver',                                 
            output='screen',                    
            condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        )
        # # 启动 vision 的 camera_init_node 节点
        # Node(
        #     package='vision',
        #     executable='down_camera_init',
        #     output='screen',
        #     name='camera_init_node',
        #     condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        # ),
        
        # # 启动 vision 的 plant_detection 节点
        # Node(
        #     package='vision',
        #     executable='plant_detection',
        #     output='screen',
        #     name='plant_detection',
        #     condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        # ),
        
        # # 启动 vision 的 color_detector 节点
        # Node(
        #     package='vision',
        #     executable='down_camera_color',
        #     output='screen',
        #     name='color_detector'
        # ),

        # Node(
        #     package='vision',
        #     executable='detection_socket_receiver',
        #     output='screen',
        #     name='detection_socket_receiver',
        #     condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        # ),
        # Node(
        #     package='extension',
        #     executable='tcpbridge',
        #     output='screen',
        #     name='tcpbridge',
        #     condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        # ),

        # 启动d435
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'realsense2_camera', 'rs_align_depth_launch.py'],
        #     output='screen',
        #     name='d435_launch',
        #     condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        # ),


        # Node(
        #     package='vision',
        #     executable='front_pole_detection',
        #     name='multi_color_pole_detector',
        #     output='screen',
        #     condition=UnlessCondition(launch.substitutions.LaunchConfiguration('simulation'))
        # ),

        # Node(
        #     package='vision',
        #     executable='ring_detector',
        #     name='ring_detector',
        #     output='screen'
        # ),
        
        # Node(
        #     package='pointcl',
        #     executable='livox_to_pcl',
        #     name='livox_to_pointcloud2',
        #     output='screen'
        # ),
        # Node(
        #     package='pointcl',
        #     executable='pole_detection',
        #     name='pole_detection_node',
        #     output='screen'
        # ),


    ])
