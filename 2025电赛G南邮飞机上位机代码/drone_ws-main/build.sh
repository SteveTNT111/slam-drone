#!/bin/bash

# 检查是否提供了参数
if [ $# -eq 0 ]; then
  echo "请提供参数：sim（仿真模式）或real（真机模式）"
  exit 1
fi

# 根据参数决定编译哪些功能包
if [ "$1" == "sim" ]; then
  echo "仿真模式：编译除cvision以外的所有功能包"
  cp ~/drone_ws/src/offboard/resource/models/iris/iris.sdf ~/PX4-Autopilot/Tools/sitl_gazebo/models/iris/
  cp ~/drone_ws/src/offboard/resource/models/depth_camera/depth_camera.sdf ~/PX4-Autopilot/Tools/sitl_gazebo/models/depth_camera/ 
  colcon build --packages-select offboard  vision msg_tool
elif [ "$1" == "real" ]; then
  echo "真机模式：编译所有功能包和QT"
  colcon build 
  # 编译 Qt 项目 PlaneScreen
  echo "开始编译 PlaneScreen Qt 项目..."

  cd PlaneScreen || { echo "错误：无法进入 PlaneScreen 目录"; exit 1; }

  # 检查构建目录
  BUILD_DIR="cmake-build-debug"
  if [ ! -d "$BUILD_DIR" ]; then
    echo "错误：构建目录 $BUILD_DIR 不存在！请先用 Qt Creator 或 cmake 创建项目配置。"
    exit 1
  fi
cd "$BUILD_DIR" || exit 1

# 如果还没有 CMakeCache.txt，需要先配置（首次编译）
if [ ! -f "CMakeCache.txt" ]; then
  echo "首次配置 CMake..."
  cmake .. \
    -DCMAKE_PREFIX_PATH="/home/kevin/Qt/5.15.2/gcc_64/lib/cmake" \
    -DCMAKE_BUILD_TYPE=Debug \
    -G "Ninja"   # 重要：使用 Ninja 生成器
fi

# 使用 ninja 编译
ninja -j$(nproc)

if [ $? -eq 0 ]; then
  echo "✅ PlaneScreen 编译成功！可执行文件位于: $(pwd)"
else
  echo "❌ PlaneScreen 编译失败！"
  exit 1
fi


else
  echo "无效的参数：$1。请使用sim或real。"
  exit 1
fi
