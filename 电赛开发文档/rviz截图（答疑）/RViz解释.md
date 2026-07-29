# RViz 解释

## 1. 这份文档是干什么的

这份文档专门解释 `rviz截图（答疑）` 文件夹里的 7 张图。

现在图片已经按阅读顺序改名为：

- `1.png`
- `2.png`
- `3.png`
- `4.png`
- `5.png`
- `6.png`
- `7.png`

阅读顺序建议也是这个顺序：

1. 先看完整界面
2. 再看基础设置
3. 再看点云、里程计、轨迹这些显示项
4. 最后再理解 RViz 只负责“可视化和交互”，不直接负责飞控控制

---

## 2. RViz 到底是什么

RViz 的英文全称可以理解为：

- `RViz`
- `ROS Visualization Tool`
- 中文：**ROS 可视化工具**

它最核心的作用有三类：

1. 看数据  
   比如点云、轨迹、姿态、地图

2. 看坐标关系  
   比如 `Fixed Frame`、`Axes`、`Odometry`

3. 发交互目标  
   比如 `2D Nav Goal`、`Publish Point`

它**不是**：

- 飞控
- 路径规划器
- 自主控制器

它更像是：

**“把 ROS 里的空间数据和目标输入变成人能看懂、能点的界面。”**

---

## 3. 图 1：完整 RViz 界面

![图1](03_竞赛资料/2026电赛D题无人机资料/slam-drone/电赛开发文档/rviz截图（答疑）/1.png)

### 3.1 这张图里你能看到哪几块

#### 左侧 `Displays`

- 英文：`Displays`
- 中文：显示项列表

这里决定：

- 你想看哪些话题
- 每种话题怎么显示
- 每种显示的颜色、大小、透明度、参考系

#### 中间黑色大区域

- 英文：3D View
- 中文：三维视图窗口

这里显示：

- 点云
- 地图
- 路径
- 坐标轴
- 姿态箭头

#### 上方工具栏

这排最常见的按钮有：

- `Interact`：交互
- `Move Camera`：移动视角
- `Select`：选择
- `Focus Camera`：聚焦视角
- `Measure`：测量
- `2D Pose Estimate`：二维初始位姿
- `2D Nav Goal`：二维导航目标
- `Publish Point`：发布一个点

#### 右侧 `Views`

- 英文：`Views`
- 中文：视角设置

当前图里是：

- `Type: Orbit (rviz)`

这表示：

- 当前是绕某个焦点旋转观察的三维视角

#### 底部状态栏

能看到：

- `ROS Time`
- `ROS Elapsed`
- `Wall Time`
- `Wall Elapsed`

这用于确认：

- ROS 时间有没有在走
- 当前显示是不是卡住了

### 3.2 试飞时这张完整界面最该怎么看

你不需要一次盯全部东西。

今天最该盯的是：

1. 中间三维视图里，地图和条带有没有撕裂飞走
2. 左边 `Displays` 里，`mapping`、`Odometry`、`Path` 有没有打开
3. `Fixed Frame` 是否合理

---

## 4. 图 2：Global Options、Grid、Axes

![图2](03_竞赛资料/2026电赛D题无人机资料/slam-drone/电赛开发文档/rviz截图（答疑）/2.png)

### 4.1 `Global Options`

- 英文：`Global Options`
- 中文：全局选项

这是 RViz 最基础的设置区。

#### `Fixed Frame`

- 英文：`Fixed Frame`
- 中文：固定参考坐标系

图里当前值是：

```text
camera_init
```

它的意思是：

**整个 RViz 都把 `camera_init` 当作世界参考系来画。**

这个概念非常重要。

如果 `Fixed Frame` 设错了，会出现：

- 画面看起来很怪
- 点云和轨迹像是在乱跳

但注意：

**`Fixed Frame` 设错会让显示变怪，不一定是系统本身真坏了。**

#### `Background Color`

- 英文：`Background Color`
- 中文：背景颜色

图里是黑色背景。

黑底看点云通常比较清楚。

#### `Frame Rate`

- 英文：`Frame Rate`
- 中文：刷新帧率

图里是 `10`。

意思是：

- RViz 以大约 10 帧每秒刷新显示

#### `Default Light`

- 英文：`Default Light`
- 中文：默认光照

这个主要影响 3D 显示视觉效果。

### 4.2 `Grid`

- 英文：`Grid`
- 中文：网格地面

图里重要字段有：

- `Reference Frame`
- `Plane Cell Count`
- `Cell Size`
- `Line Style`
- `Color`
- `Alpha`
- `Plane`

它的作用是：

**给你一个空间地面参考。**

你可以把它理解成：

- 一张悬在参考系里的网格纸

这样你看点云和轨迹时更容易判断：

- 高低
- 大小
- 倾斜方向

### 4.3 `Axes`

- 英文：`Axes`
- 中文：坐标轴

常见颜色约定：

- X：红
- Y：绿
- Z：蓝

它的作用是：

- 帮你看当前参考系方向
- 看姿态箭头朝向是否合理

---

## 5. 图 3：`mapping` 下的 `surround` 和 `currPoints`

![图3](03_竞赛资料/2026电赛D题无人机资料/slam-drone/电赛开发文档/rviz截图（答疑）/3.png)

### 5.1 `surround`

- 英文：`surround`
- 中文：周围累积点云

图里它的 `Topic` 是：

```text
/cloud_registered
```

虽然名字叫 `surround`，但当前图里显示的还是 `/cloud_registered`。

你可以把它理解成：

- 显示一段时间范围内、已经配准后的周围点云

它的显示方式：

- `Style: Points`
- `Color Transformer: Intensity`

意思是：

- 用点来显示
- 颜色跟点云强度有关

### 5.2 `currPoints`

- 英文：`currPoints`
- 中文：当前帧点云

图里它也指向：

```text
/cloud_registered
```

但参数不同：

- `Size (Pixels)` 更小
- `Decay Time` 更长
- 显示效果和 `surround` 不同

这通常是为了：

- 一层看当前帧
- 一层看累积效果

### 5.3 试飞时这张图最该看什么

如果条带飞出去，你在这里最该观察：

1. 当前帧是不是突然和地图错开
2. 点云是不是开始撕裂
3. 同一面墙是不是被画成很多层

如果这里先出问题，说明问题更靠近：

- FAST-LIO2 配准
- IMU
- 时间同步

---

## 6. 图 4：`PointCloud2` 显示 `/Laser_map`

![图4](03_竞赛资料/2026电赛D题无人机资料/slam-drone/电赛开发文档/rviz截图（答疑）/4.png)

### 6.1 这是什么

- 英文：`PointCloud2`
- 中文：点云显示

当前话题：

```text
/Laser_map
```

这一般表示：

**已经建好的地图点云。**

### 6.2 图里这些字段是什么意思

#### `Topic`

- 英文：`Topic`
- 中文：话题名

这里是：

```text
/Laser_map
```

#### `Style`

- 英文：`Style`
- 中文：显示样式

这里是：

```text
Flat Squares
```

意思是：

- 用小方块显示点云

#### `Size (m)`

- 英文：`Size (m)`
- 中文：点大小（米）

这里是 `0.1`，表示显示方块边长大概 0.1 米

#### `Color Transformer`

- 英文：`Color Transformer`
- 中文：颜色映射方式

这里是：

```text
FlatColor
```

表示：

- 所有点用统一颜色显示

### 6.3 试飞时看它能判断什么

这一层最能看出：

- 地图是不是整体稳定
- 条带是不是已经把地图拖歪
- 是否存在“地图整体飞出去”的现象

如果 `/Laser_map` 先飞，往往说明：

- 地图累积已经坏了

---

## 7. 图 5：`Odometry`

![图5](03_竞赛资料/2026电赛D题无人机资料/slam-drone/电赛开发文档/rviz截图（答疑）/5.png)

### 7.1 这是什么

- 英文：`Odometry`
- 中文：里程计显示

当前话题：

```text
/Odometry
```

这通常是 FAST-LIO2 输出的位姿。

### 7.2 重要字段解释

#### `Shape`

- 英文：`Shape`
- 中文：显示形状

这里是：

```text
Axes
```

表示把位姿画成一个坐标轴

#### `Axes Length`

- 英文：`Axes Length`
- 中文：坐标轴长度

#### `Axes Radius`

- 英文：`Axes Radius`
- 中文：坐标轴粗细

#### `Covariance`

- 英文：`Covariance`
- 中文：协方差显示

这里勾选了：

- `Position`
- `Orientation`

这表示：

- 位置不确定性
- 姿态不确定性

也会被画出来

#### `Orientation Frame`

- 英文：`Orientation Frame`
- 中文：姿态显示参考

图里是：

```text
Local
```

### 7.3 试飞时最该怎么看它

如果飞机静止时：

- 这个姿态轴自己在空间里走
- 或者方向跳来跳去

那说明：

- 里程计已经先不稳了

---

## 8. 图 6：`Path`

![图6](03_竞赛资料/2026电赛D题无人机资料/slam-drone/电赛开发文档/rviz截图（答疑）/6.png)

### 8.1 这是什么

- 英文：`Path`
- 中文：轨迹

当前话题：

```text
/path
```

它表示：

- 历史运动轨迹

### 8.2 重要字段

#### `Line Style`

- 英文：`Line Style`
- 中文：线样式

这里是：

```text
Billboards
```

表示：

- 用比较明显的线段方式显示

#### `Line Width`

- 英文：`Line Width`
- 中文：线宽

#### `Color`

- 英文：`Color`
- 中文：颜色

图里这个路径是青色。

### 8.3 试飞时怎么用它判断问题

这是一个非常实用的判断项。

如果飞机静止不动时：

- `/path` 还在自己增长

那说明：

- 定位系统自己在漂

如果起飞后：

- 轨迹突然出现不合理跳跃
- 轨迹方向和真实运动明显不一致

那说明：

- 里程计或融合链路有异常

---

## 9. 图 7：`/cloud_effected`、`/Laser_map`、`MarkerArray`

![图7](03_竞赛资料/2026电赛D题无人机资料/slam-drone/电赛开发文档/rviz截图（答疑）/7.png)

### 9.1 第一个 `PointCloud2`

当前话题：

```text
/cloud_effected
```

这个名字通常表示：

- 经过某种处理后的点云
- 可能是提取、筛选、影响区域、任务相关点云

当前显示方式：

- `Style: Spheres`
- `Color Transformer: Intensity`

说明：

- 它更像是在突出显示某种处理结果

### 9.2 第二个 `PointCloud2`

当前话题：

```text
/Laser_map
```

还是地图层。

这里的配置是：

- `Style: Flat Squares`
- `Color Transformer: FlatColor`

说明：

- 这是在用统一颜色看地图

### 9.3 `MarkerArray`

- 英文：`MarkerArray`
- 中文：标记数组

当前话题：

```text
/MarkerArray
```

它常用来显示：

- 路径点
- 目标点
- 障碍物标记
- 箭头
- 任务区域

### 9.4 为什么师兄能在 RViz 里“画箭头让飞机飞过去”

常见方式不是 MarkerArray 自己控制飞机，而是：

1. RViz 发目标
2. 后台节点收到目标
3. 后台节点把目标转换成 PX4/MAVROS setpoint
4. 飞控执行

常见来源有两个：

#### `2D Nav Goal`

会往这个话题发：

```text
/move_base_simple/goal
```

消息类型通常是：

```text
geometry_msgs/PoseStamped
```

#### `Publish Point`

会往这个话题发：

```text
/clicked_point
```

消息类型通常是：

```text
geometry_msgs/PointStamped
```

所以本质不是：

**RViz 自己控制飞机**

而是：

**RViz 发目标，后面的控制节点把目标翻译给飞控。**

---

## 10. 栅格地图、三维航线、空间坐标点航线怎么理解

### 10.1 栅格地图

- 英文：`Occupancy Grid`
- 中文：占据栅格地图

它把空间切成很多小格子，每个格子表示：

- 空闲
- 占据
- 未知

作用是：

- 给路径规划器判断哪里能走、哪里不能走

### 10.2 三维航线

最简单的三维航线，其实就是一串空间点：

```text
(x1, y1, z1, yaw1)
(x2, y2, z2, yaw2)
(x3, y3, z3, yaw3)
```

控制逻辑是：

1. 飞到第 1 个点
2. 误差小于阈值后，发第 2 个点
3. 再发第 3 个点

这就是最基础的“空间坐标系点构成的航线”。

### 10.3 RViz 在这里扮演什么角色

RViz 最适合做的是：

1. 显示地图
2. 显示路径
3. 交互式点目标点

真正的规划和控制还是要靠：

- 你自己的控制节点
- MAVROS
- PX4

---

## 11. 今天最实用的记忆方法

你现在不用把 RViz 学成一本书，先记住这几件事：

1. `PointCloud2` 看点云和地图
2. `Odometry` 看姿态和里程计方向
3. `Path` 看轨迹是不是自己长
4. `Fixed Frame` 看参考系是不是一致
5. `2D Nav Goal` 和 `Publish Point` 是“发目标”，不是“直接控飞机”

只要这 5 件事先吃透，今天试飞排查“条带为什么飞出去”就已经够用了。
