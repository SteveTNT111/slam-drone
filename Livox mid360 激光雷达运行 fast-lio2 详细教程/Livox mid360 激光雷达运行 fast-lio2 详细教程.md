# Livox mid360 激光雷达运行 fast\-lio2 详细教程

所需环境：

1、Linux版本：Ubuntu 20\.04

2、机载电脑：i3\-N305

3、激光雷达：mid\-360

# 一、运行mid\-360

## 1、硬件连接

### 1\.1将mid360的一分三航空线的网线插口插入机载电脑的网口中，然后给mid360上电。

### 1\.2

#### 1\.2\.1 进入ubuntu设置IP地址



![Livox mid360 激光雷达运行 fast\-lio2 详细教程\_449862\.png](图片和附件/Livox%20mid360%20激光雷达运行%20fast-lio2%20详细教程_449862.png)

#### 1\.2\.2 按照如图设置IP地址，子网掩码和网关（无论序列号是什么，都是这样设置，后面你就知道我为什么这么说了）

![Livox mid360 激光雷达运行 fast\-lio2 详细教程\_331382\.png](图片和附件/Livox%20mid360%20激光雷达运行%20fast-lio2%20详细教程_331382.png)

## 2、安装并运行Livox\-SDK2

### 2\.1 在主目录新建一个工作空间（专门用来放LIVOX相关，不要和fast\-lio2放一起）

```C
mkdir livox_ws
cd livox_ws
mkdir src
```

### 2\.2 安装 Livox\-SDK2

#### 2\.2\.1 安装CMake文件夹，编译驱动代码文件夹，编译驱动代码



```C
sudo apt install cmake
```

#### 2\.2\.2 编译并安装Livox\-SDK2

```C
mkdir 3rd_party
cd 3rd_party
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

提示：如果你想删除这个SDK，你可以运行这个代码（不需要的话就不用管这个）

```C
sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*sudo rm -rf /usr/local/include/livox_lidar_*
```

#### 2\.2\.3 修改host ip

进入这个文件夹，

> livox\_ws/3rd\_party/LivoxSDK2/samples/livox\_lidar\_quick\_start
> 
> 

找到mid360\_config\.json，

把 host\_ip 改成 192\.168\.1\.50

![Livox mid360 激光雷达运行 fast\-lio2 详细教程\_890836\.png](图片和附件/Livox%20mid360%20激光雷达运行%20fast-lio2%20详细教程_890836.png)

### 2\.3 运行Livox\-SDK2示例

#### 2\.3\.1

进入这个文件夹,

> livox\_ws/3rd\_party/Livox\-SDK2/build/samples/livox\_lidar\_quick\_start
> 
> 

打开终端，运行如下代码，注意：json文件和要执行的文件不在一个文件夹！！

```C
./livox_lidar_quick_start ../../../samples/livox_lidar_quick_start/mid360_config.json
```



#### 2\.3\.2 运行成功会显示以下画面，然后会有数据流一直发（如果不是这个的话可能IP错了\)

![Livox mid360 激光雷达运行 fast\-lio2 详细教程\_314535\.png](图片和附件/Livox%20mid360%20激光雷达运行%20fast-lio2%20详细教程_314535.png)

## 3、安装并运行livox\_ros\_driver2驱动

### 3\.1 下载驱动代码

```C
cd src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

### 3\.2 编译livox\_ros\_driver2驱动代码

**进入这个文件夹，在这个路径下打开终端，**

> livox\_ws/src/ws\_livox/src/livox\_ros\_driver2
> 
> 

执行以下命令，编译驱动代码

```C
./build.sh ROS1
```

### 3\.3 更改驱动json文件，

进入config文件夹，

找到MID360\_config\.json文件，

把里面的host\_net\_info的四个IP地址改成192\.168\.1\.50，

然后lidar\_configs里面的IP地址改成192\.168\.1\.1xx，

> xx为你的mid360序列号的最后两位
> 
> 

> （序列号在雷达侧面的二维码下，由于我的序列号最后两位是30，所以我改成192\.168\.1\.130，你们要根据自己的雷达序列号改）
> 
> 

![Livox mid360 激光雷达运行 fast\-lio2 详细教程\_193459\.png](图片和附件/Livox%20mid360%20激光雷达运行%20fast-lio2%20详细教程_193459.png)

### 3\.4 运行驱动

在这个文件夹打开终端：

> livox\_ws/src/ws\_livox/src/livox\_ros\_driver2
> 
> 

```C
source ../../devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch livox_ros_driver2 rviz_MID360.launch
```

如果你想之后不想source，就去主目录的\.bashrc文件自己加上去（这种ros基操就不在这说了）

### 3\.5 运行成功之后就是这样子，有点云显示就意味着你和雷达的通信是正常的啦！！！

![Livox mid360 激光雷达运行 fast\-lio2 详细教程\_119723\.png](图片和附件/Livox%20mid360%20激光雷达运行%20fast-lio2%20详细教程_119723.png)

# 二、运行fast\-lio2

## 1\.1 在主目录打开终端，执行以下命令，新开一个工作空间

```C
mkdir fast_lio2_ws
cd fast_lio2_ws
mkdir src
```

## 1\.2 下载并编译fast\-lio2，（如果你没安装eigen库和PCL库，你就得跟着源工程的readme安装一下，我这里因为安装了，所以就不写了）

```Bash
cd src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
catkin_make
source devel/setup.bash
```

提示：同样的，这里的source如果你不想之后每次都source，就去主目录的\.bashrc文件加上

## 1\.3 到这里你就会发现，你的编译不通过嘿嘿\~\~

#### 1\.4\.1 你需要来到这个文件夹

> #### fast\_lio2\_ws/src/FAST\_LIO
> 
> 

#### 找到里面的CMakeLists\.txt，把里面的livox\_ros\_driver改成livox\_ros\_driver2

![Livox mid360 激光雷达运行 fast\-lio2 详细教程\_460842\.png](图片和附件/Livox%20mid360%20激光雷达运行%20fast-lio2%20详细教程_460842.png)

#### 1\.4\.2 还需要在fastlio/src文件夹里面找到laserMapping\.cpp，把里面所有的的livox\_ros\_driver改成livox\_ros\_driver2

![Livox mid360 激光雷达运行 fast\-lio2 详细教程\_138070\.png](图片和附件/Livox%20mid360%20激光雷达运行%20fast-lio2%20详细教程_138070.png)

![Livox mid360 激光雷达运行 fast\-lio2 详细教程\_553158\.png](图片和附件/Livox%20mid360%20激光雷达运行%20fast-lio2%20详细教程_553158.png)

#### 1\.4\.3 把preprocess\.h和preprocess\.cpp文件里面的所有的的livox\_ros\_driver改成livox\_ros\_driver2

### 1\.5 现在你就可以catkin\_make了，然后source一下

### 1\.6 运行fast\-lio2

```C
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio mapping_mid360.launch
```

### 1\.7 演示效果

![Livox mid360 激光雷达运行 fast\-lio2 详细教程\_141765\.png](图片和附件/Livox%20mid360%20激光雷达运行%20fast-lio2%20详细教程_141765.png)



