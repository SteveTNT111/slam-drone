我们可以用mavros来收集飞控的数据，并想办法自动存储到ros的记录里面。
这样就可以在飞行后直接从机载电脑调取，并交给ai分析了。
飞控sd卡存储的传统日志可以存储这些mavlink消息吗？
如果你是ai读取到这些问题，请直接在下方给出回答。




password123456@ubuntu:~$ source /opt/ros/noetic/setup.bash
password123456@ubuntu:~$ roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
... logging to /home/password123456/.ros/log/9e499174-4273-11f1-b6ad-3c6d662cbb50/roslaunch-ubuntu-8025.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu:37575/

SUMMARY
========

CLEAR PARAMETERS
 * /mavros/

PARAMETERS
 * /mavros/camera/frame_id: base_link
 * /mavros/cmd/use_comp_id_system_control: False
 * /mavros/conn/heartbeat_rate: 1.0
 * /mavros/conn/system_time_rate: 1.0
 * /mavros/conn/timeout: 10.0
 * /mavros/conn/timesync_rate: 10.0
 * /mavros/distance_sensor/hrlv_ez4_pub/field_of_view: 0.0
 * /mavros/distance_sensor/hrlv_ez4_pub/frame_id: hrlv_ez4_sonar
 * /mavros/distance_sensor/hrlv_ez4_pub/id: 0
 * /mavros/distance_sensor/hrlv_ez4_pub/orientation: PITCH_270
 * /mavros/distance_sensor/hrlv_ez4_pub/send_tf: True
 * /mavros/distance_sensor/hrlv_ez4_pub/sensor_position/x: 0.0
 * /mavros/distance_sensor/hrlv_ez4_pub/sensor_position/y: 0.0
 * /mavros/distance_sensor/hrlv_ez4_pub/sensor_position/z: -0.1
 * /mavros/distance_sensor/laser_1_sub/id: 3
 * /mavros/distance_sensor/laser_1_sub/orientation: PITCH_270
 * /mavros/distance_sensor/laser_1_sub/subscriber: True
 * /mavros/distance_sensor/lidarlite_pub/field_of_view: 0.0
 * /mavros/distance_sensor/lidarlite_pub/frame_id: lidarlite_laser
 * /mavros/distance_sensor/lidarlite_pub/id: 1
 * /mavros/distance_sensor/lidarlite_pub/orientation: PITCH_270
 * /mavros/distance_sensor/lidarlite_pub/send_tf: True
 * /mavros/distance_sensor/lidarlite_pub/sensor_position/x: 0.0
 * /mavros/distance_sensor/lidarlite_pub/sensor_position/y: 0.0
 * /mavros/distance_sensor/lidarlite_pub/sensor_position/z: -0.1
 * /mavros/distance_sensor/sonar_1_sub/horizontal_fov_ratio: 1.0
 * /mavros/distance_sensor/sonar_1_sub/id: 2
 * /mavros/distance_sensor/sonar_1_sub/orientation: PITCH_270
 * /mavros/distance_sensor/sonar_1_sub/subscriber: True
 * /mavros/distance_sensor/sonar_1_sub/vertical_fov_ratio: 1.0
 * /mavros/fake_gps/eph: 2.0
 * /mavros/fake_gps/epv: 2.0
 * /mavros/fake_gps/fix_type: 3
 * /mavros/fake_gps/geo_origin/alt: 408.0
 * /mavros/fake_gps/geo_origin/lat: 47.3667
 * /mavros/fake_gps/geo_origin/lon: 8.55
 * /mavros/fake_gps/gps_rate: 5.0
 * /mavros/fake_gps/mocap_transform: True
 * /mavros/fake_gps/satellites_visible: 5
 * /mavros/fake_gps/tf/child_frame_id: fix
 * /mavros/fake_gps/tf/frame_id: map
 * /mavros/fake_gps/tf/listen: False
 * /mavros/fake_gps/tf/rate_limit: 10.0
 * /mavros/fake_gps/tf/send: False
 * /mavros/fake_gps/use_mocap: True
 * /mavros/fake_gps/use_vision: False
 * /mavros/fcu_protocol: v2.0
 * /mavros/fcu_url: /dev/ttyACM0:57600
 * /mavros/gcs_url: 
 * /mavros/global_position/child_frame_id: base_link
 * /mavros/global_position/frame_id: map
 * /mavros/global_position/gps_uere: 1.0
 * /mavros/global_position/rot_covariance: 99999.0
 * /mavros/global_position/tf/child_frame_id: base_link
 * /mavros/global_position/tf/frame_id: map
 * /mavros/global_position/tf/global_frame_id: earth
 * /mavros/global_position/tf/send: False
 * /mavros/global_position/use_relative_alt: True
 * /mavros/image/frame_id: px4flow
 * /mavros/imu/angular_velocity_stdev: 0.0003490659 // 0...
 * /mavros/imu/frame_id: base_link
 * /mavros/imu/linear_acceleration_stdev: 0.0003
 * /mavros/imu/magnetic_stdev: 0.0
 * /mavros/imu/orientation_stdev: 1.0
 * /mavros/landing_target/camera/fov_x: 2.0071286398
 * /mavros/landing_target/camera/fov_y: 2.0071286398
 * /mavros/landing_target/image/height: 480
 * /mavros/landing_target/image/width: 640
 * /mavros/landing_target/land_target_type: VISION_FIDUCIAL
 * /mavros/landing_target/listen_lt: False
 * /mavros/landing_target/mav_frame: LOCAL_NED
 * /mavros/landing_target/target_size/x: 0.3
 * /mavros/landing_target/target_size/y: 0.3
 * /mavros/landing_target/tf/child_frame_id: camera_center
 * /mavros/landing_target/tf/frame_id: landing_target
 * /mavros/landing_target/tf/listen: False
 * /mavros/landing_target/tf/rate_limit: 10.0
 * /mavros/landing_target/tf/send: True
 * /mavros/local_position/frame_id: map
 * /mavros/local_position/tf/child_frame_id: base_link
 * /mavros/local_position/tf/frame_id: map
 * /mavros/local_position/tf/send: False
 * /mavros/local_position/tf/send_fcu: False
 * /mavros/mission/pull_after_gcs: True
 * /mavros/mission/use_mission_item_int: True
 * /mavros/mocap/use_pose: True
 * /mavros/mocap/use_tf: False
 * /mavros/mount/debounce_s: 4.0
 * /mavros/mount/err_threshold_deg: 10.0
 * /mavros/mount/negate_measured_pitch: False
 * /mavros/mount/negate_measured_roll: False
 * /mavros/mount/negate_measured_yaw: False
 * /mavros/odometry/fcu/map_id_des: map
 * /mavros/odometry/fcu/odom_child_id_des: base_link
 * /mavros/odometry/fcu/odom_parent_id_des: odom
 * /mavros/plugin_blacklist: ['safety_area', '...
 * /mavros/plugin_whitelist: []
 * /mavros/px4flow/frame_id: px4flow
 * /mavros/px4flow/ranger_fov: 0.118682
 * /mavros/px4flow/ranger_max_range: 5.0
 * /mavros/px4flow/ranger_min_range: 0.3
 * /mavros/safety_area/p1/x: 1.0
 * /mavros/safety_area/p1/y: 1.0
 * /mavros/safety_area/p1/z: 1.0
 * /mavros/safety_area/p2/x: -1.0
 * /mavros/safety_area/p2/y: -1.0
 * /mavros/safety_area/p2/z: -1.0
 * /mavros/setpoint_accel/send_force: False
 * /mavros/setpoint_attitude/reverse_thrust: False
 * /mavros/setpoint_attitude/tf/child_frame_id: target_attitude
 * /mavros/setpoint_attitude/tf/frame_id: map
 * /mavros/setpoint_attitude/tf/listen: False
 * /mavros/setpoint_attitude/tf/rate_limit: 50.0
 * /mavros/setpoint_attitude/use_quaternion: False
 * /mavros/setpoint_position/mav_frame: LOCAL_NED
 * /mavros/setpoint_position/tf/child_frame_id: target_position
 * /mavros/setpoint_position/tf/frame_id: map
 * /mavros/setpoint_position/tf/listen: False
 * /mavros/setpoint_position/tf/rate_limit: 50.0
 * /mavros/setpoint_raw/thrust_scaling: 1.0
 * /mavros/setpoint_velocity/mav_frame: LOCAL_NED
 * /mavros/startup_px4_usb_quirk: False
 * /mavros/sys/disable_diag: False
 * /mavros/sys/min_voltage: 10.0
 * /mavros/target_component_id: 1
 * /mavros/target_system_id: 1
 * /mavros/tdr_radio/low_rssi: 40
 * /mavros/time/time_ref_source: fcu
 * /mavros/time/timesync_avg_alpha: 0.6
 * /mavros/time/timesync_mode: MAVLINK
 * /mavros/vibration/frame_id: base_link
 * /mavros/vision_pose/tf/child_frame_id: vision_estimate
 * /mavros/vision_pose/tf/frame_id: odom
 * /mavros/vision_pose/tf/listen: False
 * /mavros/vision_pose/tf/rate_limit: 10.0
 * /mavros/vision_speed/listen_twist: True
 * /mavros/vision_speed/twist_cov: True
 * /mavros/wheel_odometry/child_frame_id: base_link
 * /mavros/wheel_odometry/count: 2
 * /mavros/wheel_odometry/frame_id: odom
 * /mavros/wheel_odometry/send_raw: True
 * /mavros/wheel_odometry/send_twist: False
 * /mavros/wheel_odometry/tf/child_frame_id: base_link
 * /mavros/wheel_odometry/tf/frame_id: odom
 * /mavros/wheel_odometry/tf/send: False
 * /mavros/wheel_odometry/use_rpm: False
 * /mavros/wheel_odometry/vel_error: 0.1
 * /mavros/wheel_odometry/wheel0/radius: 0.05
 * /mavros/wheel_odometry/wheel0/x: 0.0
 * /mavros/wheel_odometry/wheel0/y: -0.15
 * /mavros/wheel_odometry/wheel1/radius: 0.05
 * /mavros/wheel_odometry/wheel1/x: 0.0
 * /mavros/wheel_odometry/wheel1/y: 0.15
 * /rosdistro: noetic
 * /rosversion: 1.17.4

NODES
  /
    mavros (mavros/mavros_node)

ROS_MASTER_URI=http://localhost:11311

process[mavros-1]: started with pid [8040]
[INFO] [1777320669.761807104]: FCU URL: /dev/ttyACM0:57600
[INFO] [1777320669.765580978]: serial0: device: /dev/ttyACM0 @ 57600 bps
[INFO] [1777320669.768827015]: GCS bridge disabled
[INFO] [1777320669.795550117]: Plugin 3dr_radio loaded
[INFO] [1777320669.798471445]: Plugin 3dr_radio initialized
[INFO] [1777320669.798610821]: Plugin actuator_control loaded
[INFO] [1777320669.802084756]: Plugin actuator_control initialized
[INFO] [1777320669.814424606]: Plugin adsb loaded
[INFO] [1777320669.819355284]: Plugin adsb initialized
[INFO] [1777320669.819539945]: Plugin altitude loaded
[INFO] [1777320669.820870146]: Plugin altitude initialized
[INFO] [1777320669.820988560]: Plugin cam_imu_sync loaded
[INFO] [1777320669.821729029]: Plugin cam_imu_sync initialized
[INFO] [1777320669.821844050]: Plugin camera loaded
[INFO] [1777320669.822557604]: Plugin camera initialized
[INFO] [1777320669.822667313]: Plugin cellular_status loaded
[INFO] [1777320669.825321122]: Plugin cellular_status initialized
[INFO] [1777320669.825485044]: Plugin command loaded
[INFO] [1777320669.844721495]: Plugin command initialized
[INFO] [1777320669.844994870]: Plugin companion_process_status loaded
[INFO] [1777320669.849291908]: Plugin companion_process_status initialized
[INFO] [1777320669.849634795]: Plugin debug_value loaded
[INFO] [1777320669.857536759]: Plugin debug_value initialized
[INFO] [1777320669.857607807]: Plugin distance_sensor blacklisted
[INFO] [1777320669.857786676]: Plugin esc_status loaded
[INFO] [1777320669.859121645]: Plugin esc_status initialized
[INFO] [1777320669.859261981]: Plugin esc_telemetry loaded
[INFO] [1777320669.859974639]: Plugin esc_telemetry initialized
[INFO] [1777320669.860170182]: Plugin fake_gps loaded
[INFO] [1777320669.886944298]: Plugin fake_gps initialized
[INFO] [1777320669.887212392]: Plugin ftp loaded
[INFO] [1777320669.896233493]: Plugin ftp initialized
[INFO] [1777320669.896470704]: Plugin geofence loaded
[INFO] [1777320669.900548037]: Plugin geofence initialized
[INFO] [1777320669.900787296]: Plugin global_position loaded
[INFO] [1777320669.924458176]: Plugin global_position initialized
[INFO] [1777320669.924700476]: Plugin gps_input loaded
[INFO] [1777320669.928172458]: Plugin gps_input initialized
[INFO] [1777320669.928330845]: Plugin gps_rtk loaded
[INFO] [1777320669.931712513]: Plugin gps_rtk initialized
[INFO] [1777320669.931867731]: Plugin gps_status loaded
[INFO] [1777320669.935318784]: Plugin gps_status initialized
[INFO] [1777320669.935518711]: Plugin guided_target loaded
[INFO] [1777320669.941320241]: Plugin guided_target initialized
[INFO] [1777320669.941574510]: Plugin hil loaded
[INFO] [1777320669.963353685]: Plugin hil initialized
[INFO] [1777320669.963683066]: Plugin home_position loaded
[INFO] [1777320669.969402347]: Plugin home_position initialized
[INFO] [1777320669.969620357]: Plugin imu loaded
[INFO] [1777320669.979273978]: Plugin imu initialized
[INFO] [1777320669.979495443]: Plugin landing_target loaded
[INFO] [1777320669.999427173]: Plugin landing_target initialized
[INFO] [1777320669.999979525]: Plugin local_position loaded
[INFO] [1777320670.010241602]: Plugin local_position initialized
[INFO] [1777320670.010536292]: Plugin log_transfer loaded
[INFO] [1777320670.014937567]: Plugin log_transfer initialized
[INFO] [1777320670.015124564]: Plugin mag_calibration_status loaded
[INFO] [1777320670.017022671]: Plugin mag_calibration_status initialized
[INFO] [1777320670.017546027]: Plugin manual_control loaded
[INFO] [1777320670.021647491]: Plugin manual_control initialized
[INFO] [1777320670.021831289]: Plugin mocap_pose_estimate loaded
[INFO] [1777320670.026443692]: Plugin mocap_pose_estimate initialized
[INFO] [1777320670.026989163]: Plugin mount_control loaded
[WARN] [1777320670.042964506]: Could not retrive negate_measured_roll parameter value, using default (0)
[WARN] [1777320670.043788761]: Could not retrive negate_measured_pitch parameter value, using default (0)
[WARN] [1777320670.044412065]: Could not retrive negate_measured_yaw parameter value, using default (0)
[WARN] [1777320670.046252437]: Could not retrive debounce_s parameter value, using default (4.000000)
[WARN] [1777320670.046856442]: Could not retrive err_threshold_deg parameter value, using default (10.000000)
[INFO] [1777320670.046964999]: Plugin mount_control initialized
[INFO] [1777320670.047161758]: Plugin nav_controller_output loaded
[INFO] [1777320670.048104362]: Plugin nav_controller_output initialized
[INFO] [1777320670.048266877]: Plugin obstacle_distance loaded
[INFO] [1777320670.051762511]: Plugin obstacle_distance initialized
[INFO] [1777320670.051937476]: Plugin odom loaded
[INFO] [1777320670.058184083]: Plugin odom initialized
[INFO] [1777320670.058531387]: Plugin onboard_computer_status loaded
[INFO] [1777320670.063184371]: Plugin onboard_computer_status initialized
[INFO] [1777320670.063399563]: Plugin param loaded
[INFO] [1777320670.067826281]: Plugin param initialized
[INFO] [1777320670.068113546]: Plugin play_tune loaded
[INFO] [1777320670.071883260]: Plugin play_tune initialized
[INFO] [1777320670.072281002]: Plugin px4flow loaded
[INFO] [1777320670.083342180]: Plugin px4flow initialized
[INFO] [1777320670.083680267]: Plugin rallypoint loaded
[INFO] [1777320670.088613667]: Plugin rallypoint initialized
[INFO] [1777320670.088704846]: Plugin rangefinder blacklisted
[INFO] [1777320670.089002480]: Plugin rc_io loaded
[INFO] [1777320670.092995388]: Plugin rc_io initialized
[INFO] [1777320670.093060675]: Plugin safety_area blacklisted
[INFO] [1777320670.093226102]: Plugin setpoint_accel loaded
[INFO] [1777320670.097187806]: Plugin setpoint_accel initialized
[INFO] [1777320670.097620752]: Plugin setpoint_attitude loaded
[INFO] [1777320670.113364389]: Plugin setpoint_attitude initialized
[INFO] [1777320670.113734448]: Plugin setpoint_position loaded
[INFO] [1777320670.140072425]: Plugin setpoint_position initialized
[INFO] [1777320670.140342536]: Plugin setpoint_raw loaded
[INFO] [1777320670.173963624]: Plugin setpoint_raw initialized
[INFO] [1777320670.174836428]: Plugin setpoint_trajectory loaded
[INFO] [1777320670.189590895]: Plugin setpoint_trajectory initialized
[INFO] [1777320670.190329444]: Plugin setpoint_velocity loaded
[INFO] [1777320670.206127328]: Plugin setpoint_velocity initialized
[INFO] [1777320670.206613079]: Plugin sys_status loaded
[INFO] [1777320670.226984705]: Plugin sys_status initialized
[INFO] [1777320670.227259137]: Plugin sys_time loaded
[INFO] [1777320670.233372993]: TM: Timesync mode: MAVLINK
[INFO] [1777320670.233982535]: TM: Not publishing sim time
[INFO] [1777320670.235696685]: Plugin sys_time initialized
[INFO] [1777320670.235886467]: Plugin terrain loaded
[INFO] [1777320670.236915897]: Plugin terrain initialized
[INFO] [1777320670.237130258]: Plugin trajectory loaded
[INFO] [1777320670.249016042]: Plugin trajectory initialized
[INFO] [1777320670.249299371]: Plugin tunnel loaded
[INFO] [1777320670.254057903]: Plugin tunnel initialized
[INFO] [1777320670.254413720]: Plugin vfr_hud loaded
[INFO] [1777320670.255367526]: Plugin vfr_hud initialized
[INFO] [1777320670.255473010]: Plugin vibration blacklisted
[INFO] [1777320670.255659624]: Plugin vision_pose_estimate loaded
[INFO] [1777320670.264830856]: Plugin vision_pose_estimate initialized
[INFO] [1777320670.265034751]: Plugin vision_speed_estimate loaded
[INFO] [1777320670.270335970]: Plugin vision_speed_estimate initialized
[INFO] [1777320670.270768979]: Plugin waypoint loaded
[INFO] [1777320670.277627753]: Plugin waypoint initialized
[INFO] [1777320670.277822656]: Plugin wheel_odometry blacklisted
[INFO] [1777320670.278314392]: Plugin wind_estimation loaded
[INFO] [1777320670.280367717]: Plugin wind_estimation initialized
[INFO] [1777320670.281278478]: Built-in SIMD instructions: ARM NEON
[INFO] [1777320670.281405244]: Built-in MAVLink package version: 2025.5.5
[INFO] [1777320670.281464803]: Known MAVLink dialects: common ardupilotmega ASLUAV AVSSUAS all csAirLink cubepilot development icarous loweheiser matrixpilot paparazzi standard storm32 uAvionix ualberta
[INFO] [1777320670.281502600]: MAVROS started. MY ID 1.240, TARGET ID 1.1
[INFO] [1777320674.001741665]: GF: Using MISSION_ITEM_INT
[INFO] [1777320674.001947289]: RP: Using MISSION_ITEM_INT
[INFO] [1777320674.002024834]: WP: Using MISSION_ITEM_INT
[INFO] [1777320674.002123565]: VER: 1.1: Capabilities         0x000000000000e4ff
[INFO] [1777320674.002224921]: VER: 1.1: Flight software:     010d03ff (1c8ab2a0d7000000)
[INFO] [1777320674.002307459]: VER: 1.1: Middleware software: 010d03ff (1c8ab2a0d7000000)
[INFO] [1777320674.002385228]: VER: 1.1: OS software:         0b0000ff (4a1dd8680cd29f51)
[INFO] [1777320674.002454164]: VER: 1.1: Board hardware:      00000038
[INFO] [1777320674.002532509]: VER: 1.1: VID/PID:             3185:0038
[INFO] [1777320674.002590596]: VER: 1.1: UID:                 3233511536313834
[WARN] [1777320674.004032107]: TM : RTT too high for timesync: 2168.19 ms.
[WARN] [1777320674.004641425]: TM : RTT too high for timesync: 1568.44 ms.
[WARN] [1777320674.005046080]: TM : RTT too high for timesync: 969.20 ms.
[WARN] [1777320674.005439982]: TM : RTT too high for timesync: 369.60 ms.
[INFO] [1777320674.024587327]: IMU: Attitude quaternion IMU detected!
[INFO] [1777320674.024841564]: IMU: High resolution IMU detected!
[INFO] [1777320675.001388826]: CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
[INFO] [1777320675.004503492]: IMU: Attitude quaternion IMU detected!
[INFO] [1777320675.004823977]: IMU: High resolution IMU detected!
[INFO] [1777320676.013153941]: VER: 1.1: Capabilities         0x000000000000e4ff
[INFO] [1777320676.013304903]: VER: 1.1: Flight software:     010d03ff (1c8ab2a0d7000000)
[INFO] [1777320676.013426709]: VER: 1.1: Middleware software: 010d03ff (1c8ab2a0d7000000)
[INFO] [1777320676.013528737]: VER: 1.1: OS software:         0b0000ff (4a1dd8680cd29f51)
[INFO] [1777320676.013641646]: VER: 1.1: Board hardware:      00000038
[INFO] [1777320676.013702005]: VER: 1.1: VID/PID:             3185:0038
[INFO] [1777320676.013769117]: VER: 1.1: UID:                 3233511536313834
[WARN] [1777320676.013922223]: CMD: Unexpected command 520, result 0
[INFO] [1777320685.003488514]: HP: requesting home position
[INFO] [1777320690.005538554]: GF: mission received
[INFO] [1777320690.005801370]: RP: mission received
[INFO] [1777320690.005882948]: WP: mission received
[INFO] [1777320695.003405203]: HP: requesting home position
[INFO] [1777320705.003470740]: HP: requesting home position
^C[mavros-1] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
