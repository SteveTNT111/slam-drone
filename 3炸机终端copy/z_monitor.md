password123456@ubuntu:~$ source /opt/ros/noetic/setup.bash
password123456@ubuntu:~$ python3 - <<'PY'
> import rospy
> from nav_msgs.msg import Odometry
> from geometry_msgs.msg import PoseStamped
> 
> latest = {"odom": None, "vision": None, "local": None}
> 
> def odom_cb(msg):
>     latest["odom"] = msg.pose.pose.position.z
> 
> def vision_cb(msg):
>     latest["vision"] = msg.pose.position.z
> 
> def local_cb(msg):
>     latest["local"] = msg.pose.position.z
> 
> rospy.init_node("z_snapshot", anonymous=True)
> rospy.Subscriber("/Odometry", Odometry, odom_cb)
> rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, vision_cb)
> rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_cb)
> 
> rate = rospy.Rate(2)
> for _ in range(20):
>     print("Odometry z:", latest["odom"], " | Vision z:", latest["vision"], " | Local z:", latest["local"])
>     rate.sleep()
> PY
Odometry z: None  | Vision z: None  | Local z: None
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -929147.3125
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -943294.75
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -954805.1875
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -2892090.25
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -1085964.25
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -841084.5
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -835404.75
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -862218.125
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -889164.125
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -911142.8125
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -929151.1875
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -943294.75
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -954620.6875
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -2892094.75
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -1085981.5
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -841057.1875
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -835401.4375
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -862205.25
Odometry z: None  | Vision z: -3154451.672434781  | Local z: -889156.6875
password123456@ubuntu:~$ ^C
password123456@ubuntu:~$ 




### 另一个终端，运行的是codex写的监视脚本 
2026年 04月 28日 星期二 00:46:10 PDT

[FAST-LIO z]
/bin/bash: rostopic：未找到命令

[Vision Pose z]
/bin/bash: rostopic：未找到命令

[Local Position z]
/bin/bash: rostopic：未找到命令

