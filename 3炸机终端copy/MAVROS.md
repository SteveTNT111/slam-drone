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

process[mavros-1]: started with pid [7320]
[INFO] [1777359926.072981549]: FCU URL: /dev/ttyACM0:57600
[INFO] [1777359926.077364791]: serial0: device: /dev/ttyACM0 @ 57600 bps
[INFO] [1777359926.079971286]: GCS bridge disabled
[INFO] [1777359926.140156967]: Plugin 3dr_radio loaded
[INFO] [1777359926.144673112]: Plugin 3dr_radio initialized
[INFO] [1777359926.145150469]: Plugin actuator_control loaded
[INFO] [1777359926.149917472]: Plugin actuator_control initialized
[INFO] [1777359926.161922797]: Plugin adsb loaded
[INFO] [1777359926.173919447]: Plugin adsb initialized
[INFO] [1777359926.174456982]: Plugin altitude loaded
[INFO] [1777359926.176249989]: Plugin altitude initialized
[INFO] [1777359926.176633334]: Plugin cam_imu_sync loaded
[INFO] [1777359926.177853597]: Plugin cam_imu_sync initialized
[INFO] [1777359926.178215304]: Plugin camera loaded
[INFO] [1777359926.179363962]: Plugin camera initialized
[INFO] [1777359926.179710272]: Plugin cellular_status loaded
[INFO] [1777359926.183945342]: Plugin cellular_status initialized
[INFO] [1777359926.184429645]: Plugin command loaded
[INFO] [1777359926.205241786]: Plugin command initialized
[INFO] [1777359926.205530415]: Plugin companion_process_status loaded
[INFO] [1777359926.209431243]: Plugin companion_process_status initialized
[INFO] [1777359926.209605278]: Plugin debug_value loaded
[INFO] [1777359926.217698572]: Plugin debug_value initialized
[INFO] [1777359926.217786182]: Plugin distance_sensor blacklisted
[INFO] [1777359926.217941907]: Plugin esc_status loaded
[INFO] [1777359926.220023640]: Plugin esc_status initialized
[INFO] [1777359926.220216273]: Plugin esc_telemetry loaded
[INFO] [1777359926.221536373]: Plugin esc_telemetry initialized
[INFO] [1777359926.221726413]: Plugin fake_gps loaded
[INFO] [1777359926.249909908]: Plugin fake_gps initialized
[INFO] [1777359926.250160125]: Plugin ftp loaded
[INFO] [1777359926.262153383]: Plugin ftp initialized
[INFO] [1777359926.263062675]: Plugin geofence loaded
[INFO] [1777359926.270818877]: Plugin geofence initialized
[INFO] [1777359926.271111347]: Plugin global_position loaded
[INFO] [1777359926.304613911]: Plugin global_position initialized
[INFO] [1777359926.304860352]: Plugin gps_input loaded
[INFO] [1777359926.310183615]: Plugin gps_input initialized
[INFO] [1777359926.310422853]: Plugin gps_rtk loaded
[INFO] [1777359926.318265193]: Plugin gps_rtk initialized
[INFO] [1777359926.318553406]: Plugin gps_status loaded
[INFO] [1777359926.331768399]: Plugin gps_status initialized
[INFO] [1777359926.332031453]: Plugin guided_target loaded
[INFO] [1777359926.339531132]: Plugin guided_target initialized
[INFO] [1777359926.339781317]: Plugin hil loaded
[INFO] [1777359926.367258044]: Plugin hil initialized
[INFO] [1777359926.367576570]: Plugin home_position loaded
[INFO] [1777359926.374768894]: Plugin home_position initialized
[INFO] [1777359926.375050897]: Plugin imu loaded
[INFO] [1777359926.387599807]: Plugin imu initialized
[INFO] [1777359926.387852489]: Plugin landing_target loaded
[INFO] [1777359926.419602921]: Plugin landing_target initialized
[INFO] [1777359926.419860341]: Plugin local_position loaded
[INFO] [1777359926.439284074]: Plugin local_position initialized
[INFO] [1777359926.439564156]: Plugin log_transfer loaded
[INFO] [1777359926.446738331]: Plugin log_transfer initialized
[INFO] [1777359926.446989317]: Plugin mag_calibration_status loaded
[INFO] [1777359926.449542869]: Plugin mag_calibration_status initialized
[INFO] [1777359926.449781883]: Plugin manual_control loaded
[INFO] [1777359926.453653615]: Plugin manual_control initialized
[INFO] [1777359926.453851049]: Plugin mocap_pose_estimate loaded
[INFO] [1777359926.458976237]: Plugin mocap_pose_estimate initialized
[INFO] [1777359926.459193133]: Plugin mount_control loaded
[WARN] [1777359926.467478707]: Could not retrive negate_measured_roll parameter value, using default (0)
[WARN] [1777359926.468126994]: Could not retrive negate_measured_pitch parameter value, using default (0)
[WARN] [1777359926.468712447]: Could not retrive negate_measured_yaw parameter value, using default (0)
[WARN] [1777359926.471050479]: Could not retrive debounce_s parameter value, using default (4.000000)
[WARN] [1777359926.471737817]: Could not retrive err_threshold_deg parameter value, using default (10.000000)
[INFO] [1777359926.471837590]: Plugin mount_control initialized
[INFO] [1777359926.472030319]: Plugin nav_controller_output loaded
[INFO] [1777359926.473382557]: Plugin nav_controller_output initialized
[INFO] [1777359926.473543340]: Plugin obstacle_distance loaded
[INFO] [1777359926.478029525]: Plugin obstacle_distance initialized
[INFO] [1777359926.482337929]: Plugin odom loaded
[INFO] [1777359926.501735829]: Plugin odom initialized
[INFO] [1777359926.502008678]: Plugin onboard_computer_status loaded
[INFO] [1777359926.507364718]: Plugin onboard_computer_status initialized
[INFO] [1777359926.507623226]: Plugin param loaded
[INFO] [1777359926.513191681]: Plugin param initialized
[INFO] [1777359926.513515104]: Plugin play_tune loaded
[INFO] [1777359926.521731858]: Plugin play_tune initialized
[INFO] [1777359926.527730808]: Plugin px4flow loaded
[INFO] [1777359926.616077321]: Plugin px4flow initialized
[INFO] [1777359926.616398727]: Plugin rallypoint loaded
[INFO] [1777359926.623391986]: Plugin rallypoint initialized
[INFO] [1777359926.623507187]: Plugin rangefinder blacklisted
[INFO] [1777359926.623709839]: Plugin rc_io loaded
[INFO] [1777359926.660552106]: Plugin rc_io initialized
[INFO] [1777359926.660685649]: Plugin safety_area blacklisted
[INFO] [1777359926.660908243]: Plugin setpoint_accel loaded
[INFO] [1777359926.687982323]: Plugin setpoint_accel initialized
[INFO] [1777359926.688333274]: Plugin setpoint_attitude loaded
[INFO] [1777359926.723709349]: Plugin setpoint_attitude initialized
[INFO] [1777359926.723979061]: Plugin setpoint_position loaded
[INFO] [1777359926.775135044]: Plugin setpoint_position initialized
[INFO] [1777359926.775421913]: Plugin setpoint_raw loaded
[INFO] [1777359926.805200725]: Plugin setpoint_raw initialized
[INFO] [1777359926.805483496]: Plugin setpoint_trajectory loaded
[INFO] [1777359926.812530274]: Plugin setpoint_trajectory initialized
[INFO] [1777359926.812814325]: Plugin setpoint_velocity loaded
[INFO] [1777359926.837924500]: Plugin setpoint_velocity initialized
[INFO] [1777359926.838307716]: Plugin sys_status loaded
[INFO] [1777359926.853854404]: Plugin sys_status initialized
[INFO] [1777359926.854164703]: Plugin sys_time loaded
[INFO] [1777359926.874102987]: TM: Timesync mode: MAVLINK
[INFO] [1777359926.875023386]: TM: Not publishing sim time
[INFO] [1777359926.877319070]: Plugin sys_time initialized
[INFO] [1777359926.877566374]: Plugin terrain loaded
[INFO] [1777359926.878748514]: Plugin terrain initialized
[INFO] [1777359926.878932953]: Plugin trajectory loaded
[INFO] [1777359926.887672709]: Plugin trajectory initialized
[INFO] [1777359926.887966747]: Plugin tunnel loaded
[INFO] [1777359926.894028595]: Plugin tunnel initialized
[INFO] [1777359926.894325195]: Plugin vfr_hud loaded
[INFO] [1777359926.895635660]: Plugin vfr_hud initialized
[INFO] [1777359926.895724678]: Plugin vibration blacklisted
[INFO] [1777359926.895879732]: Plugin vision_pose_estimate loaded
[INFO] [1777359926.908301948]: Plugin vision_pose_estimate initialized
[INFO] [1777359926.908563241]: Plugin vision_speed_estimate loaded
[INFO] [1777359926.924790385]: Plugin vision_speed_estimate initialized
[INFO] [1777359926.925108750]: Plugin waypoint loaded
Error in XmlRpcClient::writeRequest: write error (Connection refused).
[INFO] [1777359926.937301875]: Plugin waypoint initialized
[INFO] [1777359926.937426263]: Plugin wheel_odometry blacklisted
[INFO] [1777359926.937610350]: Plugin wind_estimation loaded
[INFO] [1777359926.938953401]: Plugin wind_estimation initialized
[INFO] [1777359926.939213125]: Built-in SIMD instructions: ARM NEON
[INFO] [1777359926.939255218]: Built-in MAVLink package version: 2025.5.5
[INFO] [1777359926.939289276]: Known MAVLink dialects: common ardupilotmega ASLUAV AVSSUAS all csAirLink cubepilot development icarous loweheiser matrixpilot paparazzi standard storm32 uAvionix ualberta
[INFO] [1777359926.939326503]: MAVROS started. MY ID 1.240, TARGET ID 1.1
[INFO] [1777359926.941131386]: IMU: Attitude quaternion IMU detected!
[INFO] [1777359926.941751344]: IMU: High resolution IMU detected!
[INFO] [1777359926.942471780]: RC_CHANNELS message detected!
[INFO] [1777359926.945450193]: CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
[INFO] [1777359926.949591284]: IMU: Attitude quaternion IMU detected!
[INFO] [1777359926.950001388]: IMU: High resolution IMU detected!
[INFO] [1777359926.950179425]: RC_CHANNELS message detected!
[INFO] [1777359927.954610461]: GF: Using MISSION_ITEM_INT
[INFO] [1777359927.954736515]: RP: Using MISSION_ITEM_INT
[INFO] [1777359927.954791796]: WP: Using MISSION_ITEM_INT
[INFO] [1777359927.954847876]: VER: 1.1: Capabilities         0x000000000000e4ff
[INFO] [1777359927.954889009]: VER: 1.1: Flight software:     010d03ff (1c8ab2a0d7000000)
[INFO] [1777359927.954920026]: VER: 1.1: Middleware software: 010d03ff (1c8ab2a0d7000000)
[INFO] [1777359927.954947010]: VER: 1.1: OS software:         0b0000ff (4a1dd8680cd29f51)
[INFO] [1777359927.954969769]: VER: 1.1: Board hardware:      00000038
[INFO] [1777359927.954995409]: VER: 1.1: VID/PID:             3185:0038
[INFO] [1777359927.955019160]: VER: 1.1: UID:                 3233511536313834
[WARN] [1777359927.955444153]: CMD: Unexpected command 520, result 0
[INFO] [1777359936.947635868]: HP: requesting home position
[INFO] [1777359941.950278132]: GF: mission received
[INFO] [1777359941.950663748]: RP: mission received
[INFO] [1777359941.950951023]: WP: mission received
[INFO] [1777359946.947604921]: HP: requesting home position
[INFO] [1777359956.948101559]: HP: requesting home position
[INFO] [1777359966.947609864]: HP: requesting home position
[INFO] [1777359976.947618730]: HP: requesting home position
[INFO] [1777359986.947600861]: HP: requesting home position
[INFO] [1777359996.947726356]: HP: requesting home position
[INFO] [1777360006.947601782]: HP: requesting home position
[INFO] [1777360016.947597122]: HP: requesting home position
[INFO] [1777360026.947711743]: HP: requesting home position
[INFO] [1777360036.947632312]: HP: requesting home position
[INFO] [1777360046.947606463]: HP: requesting home position
[INFO] [1777360056.947579327]: HP: requesting home position
[INFO] [1777360066.948689012]: HP: requesting home position
[INFO] [1777360076.947587941]: HP: requesting home position
[INFO] [1777360086.947595569]: HP: requesting home position
[INFO] [1777360096.947619355]: HP: requesting home position
[INFO] [1777360106.947599745]: HP: requesting home position
[INFO] [1777360116.947537723]: HP: requesting home position
[INFO] [1777360126.947602427]: HP: requesting home position
[INFO] [1777360136.948035240]: HP: requesting home position
[INFO] [1777360146.947606263]: HP: requesting home position
[INFO] [1777360156.947781671]: HP: requesting home position
[INFO] [1777360166.947625632]: HP: requesting home position
[INFO] [1777360176.947679364]: HP: requesting home position
[INFO] [1777360186.947585442]: HP: requesting home position
[INFO] [1777360196.947597826]: HP: requesting home position
[INFO] [1777360206.947598328]: HP: requesting home position
[INFO] [1777360216.947521498]: HP: requesting home position
[INFO] [1777360226.947621842]: HP: requesting home position
[INFO] [1777360236.947620371]: HP: requesting home position
[INFO] [1777360246.947598297]: HP: requesting home position
[INFO] [1777360256.947565842]: HP: requesting home position
[INFO] [1777360266.947549897]: HP: requesting home position
[INFO] [1777360276.952217263]: HP: requesting home position
[INFO] [1777360286.947650107]: HP: requesting home position
[INFO] [1777360296.947669867]: HP: requesting home position
[INFO] [1777360306.947605796]: HP: requesting home position
[INFO] [1777360316.947596863]: HP: requesting home position
[INFO] [1777360326.947584722]: HP: requesting home position
[INFO] [1777360336.947703581]: HP: requesting home position
[INFO] [1777360346.947595157]: HP: requesting home position
[INFO] [1777360356.947606620]: HP: requesting home position
[INFO] [1777360366.947583490]: HP: requesting home position
[INFO] [1777360376.947630207]: HP: requesting home position
[INFO] [1777360386.947604572]: HP: requesting home position
[INFO] [1777360396.947606047]: HP: requesting home position
[INFO] [1777360406.947646302]: HP: requesting home position
[INFO] [1777360416.947587020]: HP: requesting home position
[INFO] [1777360426.947737618]: HP: requesting home position
[INFO] [1777360436.947993286]: HP: requesting home position
[INFO] [1777360446.947584996]: HP: requesting home position
[INFO] [1777360456.947665210]: HP: requesting home position
[INFO] [1777360466.947636189]: HP: requesting home position
[INFO] [1777360476.947633459]: HP: requesting home position
[INFO] [1777360486.947610677]: HP: requesting home position
[INFO] [1777360496.947638526]: HP: requesting home position
[INFO] [1777360506.947596250]: HP: requesting home position
[INFO] [1777360516.947659372]: HP: requesting home position
[INFO] [1777360526.947639244]: HP: requesting home position
[INFO] [1777360536.947626040]: HP: requesting home position
[INFO] [1777360546.947616020]: HP: requesting home position
[INFO] [1777360556.947630635]: HP: requesting home position
[INFO] [1777360566.947638708]: HP: requesting home position
[INFO] [1777360576.947609566]: HP: requesting home position
[INFO] [1777360586.948197472]: HP: requesting home position
[INFO] [1777360596.947657246]: HP: requesting home position
[INFO] [1777360606.947622200]: HP: requesting home position
[INFO] [1777360616.948413045]: HP: requesting home position
[INFO] [1777360626.947600987]: HP: requesting home position
[INFO] [1777360636.947656843]: HP: requesting home position
[INFO] [1777360646.947596458]: HP: requesting home position
[INFO] [1777360656.947612309]: HP: requesting home position
[INFO] [1777360666.947592066]: HP: requesting home position
[INFO] [1777360676.955370732]: HP: requesting home position
[INFO] [1777360686.947601719]: HP: requesting home position
[INFO] [1777360696.947659324]: HP: requesting home position
[INFO] [1777360706.947585513]: HP: requesting home position
[INFO] [1777360716.947567430]: HP: requesting home position
[INFO] [1777360726.947584263]: HP: requesting home position
[INFO] [1777360736.947598352]: HP: requesting home position
[INFO] [1777360746.947595629]: HP: requesting home position
[INFO] [1777360756.947627296]: HP: requesting home position
[INFO] [1777360766.947591378]: HP: requesting home position
[INFO] [1777360776.947625635]: HP: requesting home position
[INFO] [1777360786.947607912]: HP: requesting home position
[INFO] [1777360796.947666417]: HP: requesting home position
[INFO] [1777360806.947614764]: HP: requesting home position
[INFO] [1777360816.947868203]: HP: requesting home position
[INFO] [1777360826.947611924]: HP: requesting home position
[INFO] [1777360836.947609307]: HP: requesting home position
[INFO] [1777360846.947599740]: HP: requesting home position
[INFO] [1777360856.947666169]: HP: requesting home position
[INFO] [1777360866.947623801]: HP: requesting home position
[INFO] [1777360876.947626587]: HP: requesting home position
[INFO] [1777360886.947595727]: HP: requesting home position
[INFO] [1777360896.948574438]: HP: requesting home position
[INFO] [1777360906.947649905]: HP: requesting home position
[INFO] [1777360916.947605851]: HP: requesting home position
[INFO] [1777360926.947642492]: HP: requesting home position
[INFO] [1777360936.947590449]: HP: requesting home position
[INFO] [1777360946.947608202]: HP: requesting home position
[INFO] [1777360956.947593882]: HP: requesting home position
[INFO] [1777360966.947621766]: HP: requesting home position
[INFO] [1777360976.947592098]: HP: requesting home position
[INFO] [1777360986.947583065]: HP: requesting home position
[INFO] [1777360996.947573982]: HP: requesting home position
[INFO] [1777361006.947616429]: HP: requesting home position
[INFO] [1777361016.947587358]: HP: requesting home position
[INFO] [1777361026.947602475]: HP: requesting home position
[INFO] [1777361036.947557591]: HP: requesting home position
[INFO] [1777361046.947582739]: HP: requesting home position
[INFO] [1777361056.947591044]: HP: requesting home position
[INFO] [1777361066.947577629]: HP: requesting home position
[INFO] [1777361076.947634526]: HP: requesting home position
[INFO] [1777361086.948702219]: HP: requesting home position
[INFO] [1777361096.947566686]: HP: requesting home position
[INFO] [1777361106.947605904]: HP: requesting home position
[INFO] [1777361116.947553847]: HP: requesting home position
[INFO] [1777361126.947603064]: HP: requesting home position
[INFO] [1777361136.947567863]: HP: requesting home position
[INFO] [1777361146.947583079]: HP: requesting home position
[INFO] [1777361156.947562405]: HP: requesting home position
[INFO] [1777361166.947592661]: HP: requesting home position
[INFO] [1777361176.947582987]: HP: requesting home position
[INFO] [1777361186.947564191]: HP: requesting home position
[INFO] [1777361196.947562565]: HP: requesting home position
[INFO] [1777361206.947557133]: HP: requesting home position
[INFO] [1777361216.947590343]: HP: requesting home position
[INFO] [1777361226.947599643]: HP: requesting home position
[INFO] [1777361236.947570769]: HP: requesting home position
[INFO] [1777361246.947555670]: HP: requesting home position
[INFO] [1777361256.947635605]: HP: requesting home position
[INFO] [1777361266.947562663]: HP: requesting home position
[INFO] [1777361276.947564574]: HP: requesting home position
[INFO] [1777361286.947610013]: HP: requesting home position
[INFO] [1777361296.947568032]: HP: requesting home position
[INFO] [1777361306.947565041]: HP: requesting home position
[INFO] [1777361316.947553617]: HP: requesting home position
[INFO] [1777361326.947592614]: HP: requesting home position
[INFO] [1777361336.947566539]: HP: requesting home position
[INFO] [1777361346.947668333]: HP: requesting home position
[INFO] [1777361356.947997300]: HP: requesting home position
[INFO] [1777361366.947666956]: HP: requesting home position
[INFO] [1777361376.947564043]: HP: requesting home position
[INFO] [1777361386.947568939]: HP: requesting home position
[INFO] [1777361396.947580902]: HP: requesting home position
[INFO] [1777361406.947594526]: HP: requesting home position
[INFO] [1777361416.947580208]: HP: requesting home position
[INFO] [1777361426.947575585]: HP: requesting home position
[INFO] [1777361436.947792795]: HP: requesting home position
[INFO] [1777361446.947573850]: HP: requesting home position
[INFO] [1777361456.947580164]: HP: requesting home position
[INFO] [1777361466.947566796]: HP: requesting home position
[INFO] [1777361476.947513743]: HP: requesting home position
[INFO] [1777361486.947569625]: HP: requesting home position
[INFO] [1777361496.947750632]: HP: requesting home position
[INFO] [1777361506.947615396]: HP: requesting home position
[INFO] [1777361516.947625736]: HP: requesting home position
[INFO] [1777361526.947624234]: HP: requesting home position
[INFO] [1777361536.947646094]: HP: requesting home position
[INFO] [1777361546.947613454]: HP: requesting home position
[INFO] [1777361556.947613839]: HP: requesting home position
[INFO] [1777361566.947600111]: HP: requesting home position
[INFO] [1777361576.947576046]: HP: requesting home position
[INFO] [1777361586.947599792]: HP: requesting home position
[INFO] [1777361596.947708535]: HP: requesting home position
[INFO] [1777361606.947576751]: HP: requesting home position
[INFO] [1777361616.947607153]: HP: requesting home position
[INFO] [1777361626.947606577]: HP: requesting home position
[INFO] [1777361636.947679861]: HP: requesting home position
[INFO] [1777361646.947590511]: HP: requesting home position
[INFO] [1777361656.947667987]: HP: requesting home position
[INFO] [1777361666.947620880]: HP: requesting home position
[INFO] [1777361676.947670706]: HP: requesting home position
[INFO] [1777361686.947556235]: HP: requesting home position
[INFO] [1777361696.949319571]: HP: requesting home position
[INFO] [1777361706.947655631]: HP: requesting home position
[INFO] [1777361716.990288083]: HP: requesting home position
[INFO] [1777361726.947727986]: HP: requesting home position
[INFO] [1777361736.947607434]: HP: requesting home position
[INFO] [1777361751.872338305]: HP: requesting home position
[INFO] [1777361756.947604104]: HP: requesting home position
[INFO] [1777361766.948162696]: HP: requesting home position
[INFO] [1777361776.947727136]: HP: requesting home position
[INFO] [1777361786.947610079]: HP: requesting home position
[INFO] [1777361796.947679899]: HP: requesting home position
[INFO] [1777361806.947634488]: HP: requesting home position
[INFO] [1777361816.947641468]: HP: requesting home position
[INFO] [1777361826.947648831]: HP: requesting home position
[INFO] [1777361836.947678874]: HP: requesting home position
[INFO] [1777361846.947620095]: HP: requesting home position
[INFO] [1777361856.947757019]: HP: requesting home position
[INFO] [1777361866.947651304]: HP: requesting home position
[INFO] [1777361876.947634930]: HP: requesting home position
[INFO] [1777361886.947611372]: HP: requesting home position
[INFO] [1777361896.947645199]: HP: requesting home position
[INFO] [1777361906.947651789]: HP: requesting home position
[INFO] [1777361916.947662973]: HP: requesting home position
[INFO] [1777361926.947638898]: HP: requesting home position
[INFO] [1777361936.947628453]: HP: requesting home position
[INFO] [1777361946.949778965]: HP: requesting home position
[INFO] [1777361956.947617712]: HP: requesting home position
[INFO] [1777361966.951770752]: HP: requesting home position
[INFO] [1777361976.947599773]: HP: requesting home position
[INFO] [1777361986.947676846]: HP: requesting home position
[INFO] [1777361996.947609703]: HP: requesting home position
[INFO] [1777362006.949025444]: HP: requesting home position
[INFO] [1777362016.948493945]: HP: requesting home position
[INFO] [1777362026.947615582]: HP: requesting home position
[INFO] [1777362036.947668952]: HP: requesting home position
[INFO] [1777362046.947650519]: HP: requesting home position
[INFO] [1777362056.947629263]: HP: requesting home position
[INFO] [1777362066.947616434]: HP: requesting home position
[INFO] [1777362076.947623568]: HP: requesting home position
[INFO] [1777362086.947624600]: HP: requesting home position
[INFO] [1777362096.947659866]: HP: requesting home position
[INFO] [1777362106.947597631]: HP: requesting home position
[INFO] [1777362116.947619359]: HP: requesting home position
[INFO] [1777362126.947805967]: HP: requesting home position
[INFO] [1777362136.947634555]: HP: requesting home position
^C[mavros-1] killing on exit
Debug:   mavconn: serial0: recv: v2.0 !CRC Message-Id: 111 [14 bytes] IDs: 1.1 Seq: 249
Debug:   mavconn: serial0: recv: v2.0 !CRC Message-Id: 141 [32 bytes] IDs: 1.1 Seq: 250
Debug:   mavconn: serial0: recv: v2.0 !CRC Message-Id: 30 [28 bytes] IDs: 1.1 Seq: 251
Debug:   mavconn: serial0: recv: v2.0 !CRC Message-Id: 31 [32 bytes] IDs: 1.1 Seq: 252
shutting down processing monitor...
... shutting down processing monitor complete
done
password123456@ubuntu:~$ 
