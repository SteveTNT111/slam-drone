# 无人机自主控制系统


## 系统架构

项目包含以下主要模块：

- `offboard`: 飞行控制模块
  - `offb_node`: 主要的飞行控制节点
  - `odom_to_pose_node`: 里程计数据转换节点
  - `serial_node`: 串口通信节点



## 依赖项

- ROS2
- MAVROS
- OpenCV
- Livox ROS Driver 2
- Fast-LIO

## 安装依赖

[Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)

[librealsense-dev](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

```bash
sudo apt install ros-humble-pcl-ros
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-realsense2-camera
pip install pyzbar

```

## 编译livox_ros_driver2
```bash
 cd src/livox_ros_driver2/
./build.sh humble
```

## 更换模型
```bash
cp ~/drone_ws/src/offboard/resource/models/iris/iris.sdf ~/PX4-Autopilot/Tools/sitl_gazebo/models/iris/
cp ~/drone_ws/src/offboard/resource/models/depth_camera/depth_camera.sdf ~/PX4-Autopilot/Tools/sitl_gazebo/models/depth_camera/
```

## 编译 根据情况选择
```bash
./build.sh sim
```

```bash
./build.sh real
```
## 记得更新环境变量
```bash
export PX4_DIR=~/PX4-Autopilot
export GAZEBO_PLUGIN_PATH=$PX4_DIR/build/px4_sitl_default/build_gazebo:/opt/ros/humble/lib/
export GAZEBO_MODEL_PATH=$PX4_DIR/Tools/sitl_gazebo/models:~/drone_ws/src/offboard/resource/models:~/drone_ws/src/offboard/resource/worlds/:~/drone_ws/src/livox_laser_simulation_RO2/urdf
export LD_LIBRARY_PATH=$PX4_DIR/build/px4_sitl_default/build_gazebo:${LD_LIBRARY_PATH}
```

## 启动
```bash
~/drone_ws/src/offboard/px4 --vehicle iris --world ~/drone_ws/src/offboard/resource/worlds/first.world
```

运行ros2节点，参数simulation:true/false

```bash
ros2 launch offboard start_drone.launch.py simulation:=true


or

ros2 run offboard offboard_node  --ros-args --params-file /home/kevin/drone_ws/src/offboard/config/offb_configs.yaml
```
