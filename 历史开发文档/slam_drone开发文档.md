# 开发文档

这份文档面向当前这套 `PX4 + MID360 + FAST-LIO2 + Orin NX + ROS Noetic` 开发环境，重点说明下面几件事：

1. 如何获取 NX 的 IP
2. 如何让个人计算机和 NX 建立 SSH 通信
3. 本地代码仓库修改后，如何上传到 NX
4. 产生稳定版本后，如何用 Git 做版本管理
5. 常用 Git 命令是什么意思、怎么用

---

## 1. 当前开发结构

现在这套开发方式建议分成两层：

- **个人计算机**
  - 负责写代码
  - 负责使用 Codex、VS Code、本地 Git 仓库
  - 本地仓库路径示例：
    - `D:\repos\slam-drone`

- **NX**
  - 负责连接飞控、雷达、ROS、MAVROS、FAST-LIO2
  - 负责真正运行代码
  - 工作空间路径示例：
    - `~/catkin_ws`
    - `~/livox_ws`
    - `~/fast_lio2_ws`

当前推荐原则：

- **NX 上只保留“最新版可运行代码”**
- **旧版本、试验版本、稳定版本，都交给 Git 管**

---

## 2. 如何获取 NX 的 IP

### 2.1 在 NX 上查看当前 IP

在 NX 终端里执行：

```bash
hostname -I
```

它会打印出 NX 当前的全部 IP 地址。

例如：

```text
192.168.1.50 10.234.68.127
```

这表示 NX 当前有两张网络路径：

- 一个可能是雷达网口
- 一个可能是手机热点或 Wi-Fi

### 2.2 怎么判断哪个 IP 用来 SSH

一般来说：

- **接雷达的有线网口 IP** 常常是 `192.168.1.x`
- **手机热点或 Wi-Fi 的 IP** 往往是另一个网段，比如 `10.x.x.x`

在你当前这套环境里，通常应该优先使用：

- **手机热点对应的 Wi-Fi IP**

因为：

- 它是给 NoMachine / SSH 用的
- 雷达网口容易和远程控制网络冲突

### 2.3 辅助判断方法

如果你不确定哪块网卡连的是热点，可以在 NX 上执行：

```bash
nmcli device status
ip -4 addr
```

作用：

- `nmcli device status`：看当前哪些网络设备已连接
- `ip -4 addr`：看每块网卡对应的 IPv4 地址

---

## 3. 如何确认 NX 的 SSH 服务正常

在 NX 终端里执行：

```bash
sudo systemctl status ssh --no-pager
ss -tnlp | grep :22
```

需要看到：

- `ssh.service` 是 `active (running)`
- `22` 端口处于监听状态

如果 SSH 没启动，可以执行：

```bash
sudo apt update
sudo apt install -y openssh-server
sudo systemctl enable ssh
sudo systemctl restart ssh
```

---

## 4. 如何让个人计算机和 NX 建立 SSH 通信

### 4.1 最基本的 SSH 连接命令

在个人计算机的 PowerShell 里执行：

```powershell
ssh 用户名@NX的IP
```

当前这台 NX 的用户名示例是：

```text
password123456
```

如果当前热点 IP 是 `10.234.68.127`，那么命令就是：

```powershell
ssh password123456@10.234.68.127
```

### 4.2 快速测试 SSH 是否真的打通

```powershell
ssh password123456@10.234.68.127 "echo SSH_OK"
```

如果输出：

```text
SSH_OK
```

说明：

- SSH 服务是通的
- 网络路径是通的
- 用户名和目标 IP 也是对的

### 4.3 如果 SSH 连不上，先检查这几项

1. NX 和个人计算机是否连到了同一个手机热点
2. NX 的 SSH 服务是否已启动
3. 你用的是不是 **热点 Wi-Fi IP**，而不是雷达网口 IP
4. 雷达插上后是否又把网络路由搞乱了

---

## 5. 如何检查 NoMachine 是否正常

如果飞机还能联网，但 NoMachine 连不上，先不要急着重装。  
优先在 NX 终端里执行下面这些检查命令。

### 5.1 查看 NoMachine 服务状态

```bash
sudo /usr/NX/bin/nxserver --status
```

作用：

- 看 NoMachine 服务是不是正常运行
- 看当前有没有会话

### 5.2 查看 NoMachine 端口是否在监听

```bash
ss -tnlp | grep :4000
```

作用：

- 检查 NoMachine 默认端口 `4000` 是否在监听

如果有输出，通常说明：

- NoMachine 服务至少已经起来了

### 5.3 查看 NoMachine 相关进程

```bash
ps -ef | grep nx
```

作用：

- 看 `nxserver`、`nxnode`、`nxagent` 之类的进程还在不在

### 5.4 查看 NoMachine 最近日志

```bash
sudo tail -n 50 /usr/NX/var/log/nxserver.log
```

作用：

- 快速看最近有没有报错
- 常见问题包括端口冲突、会话异常、网络断开

### 5.5 如果 NoMachine 状态不对，尝试重启

```bash
sudo /usr/NX/bin/nxserver --restart
sudo /usr/NX/bin/nxserver --status
```

作用：

- 重启 NoMachine 服务
- 再次确认服务状态

### 5.6 从个人计算机测试 NoMachine 端口通不通

在个人计算机的 PowerShell 里执行：

```powershell
Test-NetConnection 10.234.68.127 -Port 4000
```

如果 `TcpTestSucceeded` 是 `True`，说明：

- NX 侧端口可达
- 问题更像是 NoMachine 会话或图形侧异常

如果是 `False`，说明：

- 端口没有监听
- 或者网络路径有问题

### 5.7 从个人计算机测试 SSH 端口通不通

```powershell
Test-NetConnection 10.234.68.127 -Port 22
```

如果这里是 `True`，但 `4000` 不通，说明：

- 飞机网络还在
- 可以先走 SSH 推代码和查日志
- NoMachine 可以后面再修

---

## 6. 本地代码修改后，如何上传到 NX

这里分两种方式：

1. **推荐方式：用中文批处理脚本**
2. **手动方式：直接用 `scp`**

### 6.1 推荐方式：用中文批处理脚本

本地已经准备好的批处理脚本路径：

- `D:\repos\slam-drone\连接NX并上传工作空间.bat`

你可以直接在资源管理器里双击它，也可以在 PowerShell 里执行：

```powershell
cmd /c "D:\repos\slam-drone\连接NX并上传工作空间.bat"
```

补充说明：

- 这个批处理脚本为了避免 Windows `cmd` 对中文脚本编码的兼容问题，**内部菜单提示使用英文**
- 但实际功能和操作步骤，已经在这份开发文档里用中文写清楚
- 你实际使用时，只需要输入 IP，然后按数字菜单操作即可

这个批处理脚本会按下面顺序工作：

1. 让你输入当前 NX 的 IP 地址  
   例如你当前常用的是：

   ```text
   10.234.68.127
   ```

2. 自动用 SSH 测试这台 NX 能不能连通
3. 弹出数字菜单，让你选择：
   - `1`：直接上传当前工作空间
   - `2`：更改本地工作空间路径并保存
   - `3`：重新输入 NX IP
   - `4`：退出
4. 如果你选择 `1`，脚本会自动：
   - 连接 NX
   - 创建 `~/catkin_ws/src` 和 `~/catkin_ws/tools`
   - 上传本地工作空间里的 `src`
   - 上传本地工作空间里的 `tools`
   - 上传工作空间根目录下的 `.md` 说明文档
   - 自动补脚本执行权限

### 6.1.1 默认保存的工作空间路径

批处理脚本会把工作空间路径保存在这个文件里：

- `D:\repos\slam-drone\nx_upload_config.txt`

当前默认路径是：

```text
D:\repos\slam-drone\catkin_ws
```

如果是你自己的电脑，通常不需要改。  
如果别人把这套脚本复制到他们自己的电脑上，只要在菜单里输入：

```text
2
```

然后把他们自己的工作空间路径粘贴进去，脚本就会自动保存，下次不用再改。

### 6.1.2 典型使用方法

#### 场景 1：你自己日常使用

1. 双击：

   - `连接NX并上传工作空间.bat`

2. 粘贴当前 NX IP，例如：

   ```text
   10.234.68.127
   ```

3. 等 SSH 测试通过
4. 输入：

   ```text
   1
   ```

5. 等待上传完成

#### 场景 2：换一台电脑或换一个本地路径

1. 双击：

   - `连接NX并上传工作空间.bat`

2. 输入当前 NX IP
3. 在菜单里输入：

   ```text
   2
   ```

4. 粘贴新的本地工作空间路径
5. 保存成功后，再输入：

   ```text
   1
   ```

进行上传

### 6.1.3 为什么现在推荐这个批处理脚本

它解决的是你之前最烦的两件事：

1. **NoMachine 重连后 IP 老变，每次都要重新问上传命令**
2. **不同电脑上的本地工作空间路径不一样**

现在这两个问题都交给这个批处理脚本自己处理，不需要你每次再手动改命令。

### 6.2 手动方式：直接用 `scp`

如果你不想用脚本，也可以直接手动传。

#### 第一步：先在 NX 上创建目录

```powershell
ssh password123456@10.234.68.127 "mkdir -p ~/catkin_ws/src ~/catkin_ws/tools"
```

#### 第二步：推整个 `fastlio_to_mavros` 包

```powershell
scp -r "D:/repos/slam-drone/catkin_ws/src/fastlio_to_mavros" password123456@10.234.68.127:~/catkin_ws/src/
```

#### 第三步：推一键启动脚本

```powershell
scp "D:/repos/slam-drone/catkin_ws/tools/start_uav_stack.sh" password123456@10.234.68.127:~/catkin_ws/tools/
```

#### 第四步：如果只改了桥接脚本，也可以只推单个文件

```powershell
scp "D:/repos/slam-drone/catkin_ws/src/fastlio_to_mavros/scripts/fastlio_mavros_bridge.py" password123456@10.234.68.127:~/catkin_ws/src/fastlio_to_mavros/scripts/
```

#### 第五步：给脚本执行权限

在 NX 上执行：

```bash
chmod +x ~/catkin_ws/tools/start_uav_stack.sh
chmod +x ~/catkin_ws/src/fastlio_to_mavros/scripts/fastlio_mavros_bridge.py
```

---

## 7. 上传代码后，NX 端怎么运行

### 6.1 手动启动桥接脚本

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
python3 ~/catkin_ws/src/fastlio_to_mavros/scripts/fastlio_mavros_bridge.py
```

### 6.2 一键启动四条链路

```bash
bash ~/catkin_ws/tools/start_uav_stack.sh
```

说明：

- 这个脚本建议在 **NX 的 NoMachine 图形终端** 里运行
- 它默认按顺序弹出 4 个独立终端窗口

### 6.3 改完代码后要不要重新编译

#### 一般不用重新编译的情况

- 只改了 `.py`
- 只改了 `.launch`
- 只改了 `.yaml`

这时通常执行：

```bash
source ~/catkin_ws/devel/setup.bash
```

然后重启对应节点就行。

#### 需要重新编译的情况

- 改了 `CMakeLists.txt`
- 改了 `package.xml`
- 改了 C++ 源码

这时执行：

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

---

## 8. Git 的基本概念

这里用最直接的话解释。

### 7.1 什么是 Git

Git 是一个**版本控制工具**。

它负责解决这些问题：

- 代码改了什么
- 谁改的
- 什么时候改的
- 哪个版本是稳定版
- 改坏了怎么回去

### 7.2 什么是仓库

仓库就是一个**被 Git 管起来的文件夹**。

例如：

- `D:\repos\slam-drone`

这个文件夹里会有一个隐藏目录：

- `.git`

里面保存的是版本历史。

### 7.3 什么是 commit

`commit` 可以理解成：

**给当前代码拍一个快照，并写一句说明。**

比如：

- “恢复 bridge 脚本 v1”
- “第一次稳定悬停前的版本”
- “修正桥接脚本注释和启动脚本”

### 7.4 什么是 branch

`branch` 是分支，可以理解成：

**一条并行开发线。**

例如：

- `main`：稳定主线
- `test-hover`：测试室内定点
- `qr-scan`：测试二维码识别

### 7.5 什么是 push 和 pull

- `push`：把本地仓库的提交推到远程仓库
- `pull`：把远程仓库的新提交拉回本地

### 7.6 什么是 GitHub

GitHub 不是 Git 本体。

GitHub 是：

- 远程托管平台
- 云端备份位置
- 协作平台

你完全可以：

- 先只用本地 Git
- 后面再接 GitHub

---

## 9. 当前项目的推荐 Git 管理方式

### 8.1 推荐管理对象

建议 Git 管这些内容：

- `catkin_ws/src/fastlio_to_mavros`
- `catkin_ws/tools/start_uav_stack.sh`
- 其他你自己写的 ROS 包
- 启动脚本
- 说明文档

### 8.2 不建议放进 Git 的内容

不要把这些东西塞进仓库：

- `build/`
- `devel/`
- `log/`
- `.bag`
- 大地图点云
- 临时输出文件

### 8.3 当前推荐工作流

最适合你现在的节奏是：

1. 在个人计算机上改代码
2. 用 Git 提交一个版本点
3. 通过 `scp` 或同步脚本推到 NX
4. 在 NX 上运行测试
5. 如果稳定，再打标签或继续提交

---

## 10. Git 最常用命令和中文解释

### 9.1 查看当前状态

```powershell
cd D:\repos\slam-drone
git status
```

作用：

- 看哪些文件被改了
- 看哪些文件还没加入提交
- 看当前分支情况

### 9.2 查看简洁状态

```powershell
git status -sb
```

作用：

- 更紧凑地看改动
- 适合日常快速检查

### 9.3 把改动加入暂存区

#### 加入整个包

```powershell
git add catkin_ws/src/fastlio_to_mavros
```

#### 加入启动脚本

```powershell
git add catkin_ws/tools/start_uav_stack.sh
```

#### 加入开发文档

```powershell
git add catkin_ws/开发文档.md
```

作用：

- 告诉 Git：这些改动我要放进下一次提交里

### 9.4 提交一个版本点

```powershell
git commit -m "恢复 fastlio_to_mavros v1，整理启动脚本和文档"
```

作用：

- 生成一个可回滚的版本点

### 9.5 查看提交历史

```powershell
git log --oneline --decorate -10
```

作用：

- 看最近 10 次提交
- 看每次提交的简短说明

### 9.6 查看当前改了什么

```powershell
git diff
```

作用：

- 看还没提交的改动

### 9.7 取消某个文件的未提交修改

```powershell
git restore 路径
```

例如：

```powershell
git restore catkin_ws/src/fastlio_to_mavros/scripts/fastlio_mavros_bridge.py
```

作用：

- 丢掉这个文件当前没提交的修改

注意：

- 这个命令会直接丢失未提交内容

### 9.8 给稳定版本打标签

当你觉得某个版本比较稳定时，可以打标签：

```powershell
git tag -a v1-stable -m "第一次恢复试飞稳定版本"
```

查看标签：

```powershell
git tag
```

作用：

- 给重要稳定版本起一个清晰名字
- 以后回看、回滚更方便

### 9.9 临时查看旧版本

```powershell
git checkout 提交号
```

例如：

```powershell
git checkout abc1234
```

回到主线：

```powershell
git checkout main
```

注意：

- 如果你的主分支不是 `main`，就换成实际分支名

---

## 11. 什么时候适合提交一个版本点

下面这些时机都很适合 `commit`：

- 刚把一套老代码成功恢复出来
- 桥接脚本能正常跑了
- 四终端启动链路跑通了
- 第一次拴绳试飞比较稳定
- 某次参数调整明显更好

一个简单原则：

**每做完一个逻辑完整的小修改，就提交一次。**

不要等改了一大堆、自己都记不清的时候再提交。

---

## 12. 推荐的最小 Git 使用流程

假设你刚修完桥接脚本，想把它记成一个稳定点：

### 第一步：查看状态

```powershell
cd D:\repos\slam-drone
git status
```

### 第二步：加入这次想提交的文件

```powershell
git add catkin_ws/src/fastlio_to_mavros
git add catkin_ws/tools/start_uav_stack.sh
git add catkin_ws/常用启动命令.md
git add catkin_ws/开发文档.md
```

### 第三步：提交

```powershell
git commit -m "恢复 bridge v1，整理启动脚本和中文开发文档"
```

### 第四步：可选，给稳定版本打标签

```powershell
git tag -a v1-stable -m "第一次恢复试飞稳定版本"
```

### 第五步：把本地最新版推到 NX 测试

```powershell
cmd /c "D:\repos\slam-drone\连接NX并上传工作空间.bat"
```

---

## 13. 以后接入 GitHub 时的大致流程

现在你可以先不急着接 GitHub，但后面推荐这样做：

1. 本地仓库先管理好
2. 建一个私有 GitHub 仓库
3. 把本地仓库推上去
4. 后面用 GitHub 做远程备份和协作

常见命令会是：

```powershell
git remote add origin 你的GitHub仓库地址
git branch -M main
git push -u origin main
```

以后再推新版本：

```powershell
git push
```

以后从 GitHub 拉更新：

```powershell
git pull --ff-only
```

---

## 14. 当前最重要的实践建议

对这个项目来说，最重要的不是把 Git 学成一本书，而是先形成这 4 个习惯：

1. **改代码前先看 `git status`**
2. **逻辑完整的小修改做一次 `commit`**
3. **稳定版本打标签**
4. **NX 只放最新版，历史交给 Git**

这样你后面做：

- 室内定点
- 避障
- 目标识别
- 航线飞行

都会轻松很多。

---

## 15. 当前 PX4 飞控侧的关键背景配置

根据当前项目的既有做法，飞控侧有几个非常关键的背景前提：

### 14.1 推荐固件版本

当前经验上更稳定的实验固件是：

- `PX4 1.13.3`
- 对应板卡文件常见为：
  - `px4-fmuv6c`

### 14.2 你们当前的 EKF2 关键参数

在地面站/QGC 中，你们当前主要依赖这两个设置：

#### `EKF2_AID_MASK = 24`

目的：

- 在无 GPS 情况下，仍然允许飞控依赖外部视觉/雷达位姿进入 `Position` 模式

#### `EKF2_HGT_MODE = vision`

目的：

- 让飞控高度估计更多依赖视觉/雷达高度

### 14.3 这两个参数带来的直接后果

一旦这样配置，试飞前就不能只看：

- 雷达有没有点云
- FAST-LIO2 有没有出 `/Odometry`

还必须额外确认：

1. 飞控是否真的收到了 `/mavros/vision_pose/pose`
2. 飞控融合后的本地位置有没有正常更新
3. 高度 z 是否稳定
4. 在无 GPS 情况下，`Position` 模式是否真的能切进去

换句话说：

- **算法侧正常**
- **不等于飞控融合侧正常**

这也是为什么你们后面试飞前一定要同时看：

- 地面站
- RViz
- 终端话题

---

## 16. 当前项目推荐的起飞前检查顺序

结合你们现在的 PX4 参数设置，推荐按下面顺序做飞前检查：

### 第一步：在地面站确认参数

确认：

- 固件版本正确
- `EKF2_AID_MASK = 24`
- `EKF2_HGT_MODE = vision`
- 没有明显 EKF 相关报警

### 第二步：在 ROS 侧确认链路

```bash
rostopic echo -n 1 /mavros/state
rostopic hz /Odometry
rostopic hz /mavros/vision_pose/pose
```

### 第三步：重点看 z 值

```bash
rostopic echo /Odometry
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```

### 第四步：在 RViz 里看空间关系

看：

- `Odometry`
- `Path`
- `curr_points`
- `surround`
- `/Laser_map`

### 第五步：地面试切 Position 模式

如果切不进去，或者切进去马上退出，说明当前“无 GPS + 外部视觉定位”条件还没满足，先不要起飞。

---

## 17. ROS 录包与数据回传

试飞时，尽量不要再靠手工复制终端输出。  
更稳的办法是直接录 `rosbag`，这样可以把带时间戳的话题完整保存下来，后面慢慢分析。

### 16.1 轻量包和完整包的区别

#### 轻量包 `light`

适合每次试飞都开，用来判断：

- 飞控模式有没有切对
- 遥控输入有没有异常
- `/Odometry`、`/mavros/vision_pose/pose`、`/mavros/local_position/pose` 三条链谁先坏

当前轻量包会记录：

- `/mavros/state`
- `/mavros/extended_state`
- `/mavros/imu/data_raw`
- `/mavros/rc/in`
- `/mavros/rc/out`
- `/mavros/battery`
- `/Odometry`
- `/mavros/vision_pose/pose`
- `/mavros/local_position/pose`
- `/mavros/local_position/velocity_local`
- `/mavros/setpoint_raw/attitude`
- `/px4ctrl/takeoff_land`
- `/debugPx4ctrl`
- `/position_cmd`
- `/traj_start_trigger`
- `/path`
- `/tf`
- `/tf_static`

#### 完整包 `full`

适合排查：

- 条带飞出去
- 点云突然乱掉
- FAST-LIO2 是否先坏

完整包会在轻量包基础上，额外记录：

- `/livox/lidar`
- `/livox/imu`
- `/cloud_registered`
- `/Laser_map`

### 16.2 手动开始录包

#### 轻量包

```bash
bash ~/catkin_ws/tools/record_flight_debug.sh light
```

#### 完整包

```bash
bash ~/catkin_ws/tools/record_flight_debug.sh full
```

如果不写参数，默认也是：

```bash
bash ~/catkin_ws/tools/record_flight_debug.sh
```

默认等价于：

```text
light
```

#### 定点模式悬停包

只想先验证 PX4 原生 `Position/Altitude` 类模式是否能靠 FAST-LIO2 外部视觉稳定悬停时，用：

```bash
bash ~/catkin_ws/tools/collect_position_hover_data.sh
```

这个脚本只录包，不会切 `OFFBOARD`，也不会触发 px4ctrl 起飞。落地停止后分析：

```bash
bash ~/catkin_ws/tools/analyze_hover_bag.sh latest --target-z 1.0
```

分析结果会放在：

```bash
~/catkin_ws/rosbags/analysis/包名/
```

### 16.3 一键启动时自动录包

当前一键启动脚本已经集成了录包窗口，默认会自动启动：

```bash
bash ~/catkin_ws/tools/record_flight_debug.sh light
```

也就是说，你现在直接运行：

```bash
bash ~/catkin_ws/tools/start_uav_stack.sh
```

除了原来的系统终端，还会自动多开一个录包终端。

### 16.4 停止录包

落地并断桨后，在录包窗口按：

```bash
Ctrl+C
```

### 16.5 NX 上录包文件保存在哪里

```bash
~/catkin_ws/rosbags
```

本地仓库里也保留了一个对应目录，便于拉回后统一管理：

```text
D:\repos\slam-drone\catkin_ws\rosbags
```

### 16.6 从个人计算机拉回录包

```powershell
scp -r password123456@你的NX热点IP:~/catkin_ws/rosbags "D:/repos/slam-drone/catkin_ws/"
```

### 16.7 把新版录包和监视脚本推到 NX

```powershell
ssh password123456@你的NX热点IP "mkdir -p ~/catkin_ws/tools ~/catkin_ws/rosbags"
scp "D:/repos/slam-drone/catkin_ws/tools/start_uav_stack.sh" password123456@你的NX热点IP:~/catkin_ws/tools/
scp "D:/repos/slam-drone/catkin_ws/tools/record_flight_debug.sh" password123456@你的NX热点IP:~/catkin_ws/tools/
scp "D:/repos/slam-drone/catkin_ws/tools/monitor_flight_debug.sh" password123456@你的NX热点IP:~/catkin_ws/tools/
scp "D:/repos/slam-drone/catkin_ws/tools/collect_position_hover_data.sh" password123456@你的NX热点IP:~/catkin_ws/tools/
scp "D:/repos/slam-drone/catkin_ws/tools/analyze_hover_bag.py" password123456@你的NX热点IP:~/catkin_ws/tools/
scp "D:/repos/slam-drone/catkin_ws/tools/analyze_hover_bag.sh" password123456@你的NX热点IP:~/catkin_ws/tools/
scp "D:/repos/slam-drone/catkin_ws/tools/nx_one_click_start.sh" password123456@你的NX热点IP:~/catkin_ws/tools/
ssh password123456@你的NX热点IP "chmod +x ~/catkin_ws/tools/start_uav_stack.sh ~/catkin_ws/tools/record_flight_debug.sh ~/catkin_ws/tools/monitor_flight_debug.sh ~/catkin_ws/tools/collect_position_hover_data.sh ~/catkin_ws/tools/analyze_hover_bag.py ~/catkin_ws/tools/analyze_hover_bag.sh ~/catkin_ws/tools/nx_one_click_start.sh"
```

### 16.8 为什么这次要优先录包

这次你们遇到的是：

- 高度模式下飞机自己缓慢上升
- RViz 里的条带不是正常随高度抬升，而是直接往斜上方飞出去

这种问题只看终端截屏不够，必须依靠带时间戳的话题记录，才能判断：

1. 是 `/Odometry` 先坏
2. 还是 `/mavros/vision_pose/pose` 先坏
3. 还是 `/mavros/local_position/pose` 先坏
4. 条带飞出去是在空中先发生，还是摔机瞬间才发生

---

## 18. 如何查看 NX 的 CPU、GPU、内存和温度

这部分主要用于：

- 试飞前确认小电脑负载是否过高
- 怀疑 FAST-LIO2、Livox、MAVROS 把小电脑跑满时快速定位
- 查看温度是否过高，是否存在降频风险

### 18.1 最常用：`tegrastats`

Jetson 平台最直接、最实用的状态查看命令是：

```bash
tegrastats
```

这个命令会持续刷新，通常能看到：

- CPU 各核心占用
- GPU 占用
- 内存占用
- 温度
- 功耗

停止方法：

```bash
Ctrl+C
```

如果想每隔 1 秒输出一次，可以用：

```bash
tegrastats --interval 1000
```

### 18.2 更好看：`jtop`

如果 NX 上还没装 `jtop`，先安装：

```bash
sudo apt update
sudo apt install -y python3-pip
sudo pip3 install -U jetson-stats
sudo reboot
```

重启后运行：

```bash
jtop
```

`jtop` 适合看：

- CPU 占用曲线
- GPU 占用
- 内存和 swap
- 温度
- 电源模式
- 风扇状态

如果你想快速判断“小电脑是不是已经跑满了”，`jtop` 通常比纯命令行更直观。

### 18.3 只看内存

```bash
free -h
```

作用：

- 看内存总量、已用、剩余
- 判断是不是内存吃满导致系统变卡

### 18.4 只看进程占用

```bash
top
```

如果系统装了 `htop`，更推荐：

```bash
htop
```

如果没有 `htop`，安装：

```bash
sudo apt update
sudo apt install -y htop
```

作用：

- 看哪个进程最吃 CPU
- 看是不是 `laserMapping`、`livox_ros_driver2`、`rviz` 或别的节点把机器跑满

### 18.5 只看温度

Jetson 上最通用的办法之一是直接看热区：

```bash
for f in /sys/class/thermal/thermal_zone*/temp; do echo "$f : $(cat $f)"; done
```

这个输出通常是毫摄氏度，例如：

```text
42000
```

表示：

```text
42.0 摄氏度
```

如果你想看得更清楚一点：

```bash
for f in /sys/class/thermal/thermal_zone*/temp; do awk '{printf "%.1f C\n", $1/1000}' "$f"; done
```

### 18.6 只在 SSH 里快速看一次

如果你只是远程通过 SSH 想快速确认状态，可以直接执行：

```bash
tegrastats --interval 1000
```

看几秒后按：

```bash
Ctrl+C
```

这通常已经足够判断：

- CPU 是否接近跑满
- GPU 是否有明显负载
- 温度是否异常高

### 18.7 Jetson 上不要优先指望 `nvidia-smi`

很多人会下意识敲：

```bash
nvidia-smi
```

但在 Jetson/Orin NX 上，它通常不是首选方法，很多情况下也不会像桌面显卡那样工作正常。  
对你现在这套平台，优先顺序建议是：

1. `tegrastats`
2. `jtop`
3. `top / htop`
4. `/sys/class/thermal`

### 18.8 试飞前推荐怎么用

试飞前你可以先开一个额外终端，执行：

```bash
tegrastats --interval 1000
```

然后再启动：

- Livox
- FAST-LIO2
- bridge
- RViz

观察：

- CPU 是否长期接近 100%
- GPU 是否异常高
- 温度是否快速上升

如果一启动系统就已经高负载、温度飙升，那么试飞时出现：

- 条带乱飞
- RViz 卡顿
- 话题频率抖动

就不一定全是算法问题，也可能是小电脑本身已经超负荷了。
