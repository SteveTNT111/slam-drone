cmake_minimum_required(VERSION 3.0.2)
project(px4_basic_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/one_key_takeoff_hover_land.py
  scripts/nupt_style_takeoff_0p3.py
  scripts/nupt_style_forward_back_0p3.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/run_protected_takeoff_0p3.sh
  scripts/run_protected_forward_back_0p3.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_safety_math.py)
  catkin_add_nosetests(test/test_nupt_descent_safety.py)
  catkin_add_nosetests(test/test_nupt_forward_back.py)
  add_rostest(test/one_key_takeoff_hover_land_dry_run.test)
  add_rostest(test/one_key_takeoff_hover_land.test)
  add_rostest(test/one_key_takeoff_hover_land_takeover.test)
  add_rostest(test/one_key_takeoff_hover_land_timeout.test)
  add_rostest(test/one_key_takeoff_hover_land_auto_land_rejection.test)
  add_rostest(test/one_key_takeoff_hover_land_failsafe_paths.test)
  add_rostest(test/one_key_takeoff_hover_land_timestamp_skew.test)
  add_rostest(test/one_key_takeoff_hover_land_setpoint_continuity.test)
endif()
