password123456@ubuntu:~$ source /opt/ros/noetic/setup.bash
password123456@ubuntu:~$ rostopic hz /Odometry /mavros/vision_pose/pose /mavros/local_position/pose
subscribed to [/Odometry]
subscribed to [/mavros/vision_pose/pose]
subscribed to [/mavros/local_position/pose]
           topic               rate   min_delta   max_delta    std_dev    window
================================================================================
/mavros/vision_pose/pose      29.99   0.03091     0.03562     0.0007106   30    
/mavros/local_position/pose   29.98   0.03244     0.03606     0.0008229   30    

           topic               rate   min_delta   max_delta    std_dev    window
================================================================================
/mavros/vision_pose/pose      30.0    0.02993     0.03758     0.0009361   60    
/mavros/local_position/pose   30.01   0.02782     0.03901     0.001537    60    

           topic               rate   min_delta   max_delta   std_dev    window
===============================================================================
/mavros/vision_pose/pose      30.0    0.02615     0.0377      0.00135    90    
/mavros/local_position/pose   29.98   0.02782     0.03901     0.001497   90    

           topic               rate   min_delta   max_delta   std_dev    window
===============================================================================
/mavros/vision_pose/pose      29.98   0.02615     0.0377      0.001263   120   
/mavros/local_position/pose   29.98   0.02782     0.03901     0.001412   120   

           topic               rate   min_delta   max_delta   std_dev    window
===============================================================================
/mavros/vision_pose/pose      30.0    0.02615     0.0377      0.001331   150   
/mavros/local_position/pose   29.99   0.02782     0.04035     0.001627   150   

           topic              rate   min_delta   max_delta   std_dev    window
==============================================================================
/mavros/vision_pose/pose      30.0   0.02615     0.0377      0.001279   180   
/mavros/local_position/pose   30.0   0.027       0.04035     0.001738   180   

           topic              rate   min_delta   max_delta   std_dev    window
==============================================================================
/mavros/vision_pose/pose      30.0   0.02615     0.0377      0.001237   210   
/mavros/local_position/pose   30.0   0.027       0.04035     0.001674   210   

           topic              rate   min_delta   max_delta   std_dev    window
==============================================================================
/mavros/vision_pose/pose      30.0   0.02615     0.03826     0.001271   240   
/mavros/local_position/pose   30.0   0.027       0.04035     0.001612   240   

           topic              rate   min_delta   max_delta   std_dev    window
==============================================================================
/mavros/vision_pose/pose      30.0   0.02615     0.03826     0.001232   270   
/mavros/local_position/pose   30.0   0.027       0.04035     0.001675   270   

           topic              rate   min_delta   max_delta   std_dev    window
==============================================================================
/mavros/vision_pose/pose      30.0   0.02615     0.03826     0.0012     300   
/mavros/local_position/pose   30.0   0.027       0.04035     0.001633   300   

           topic              rate   min_delta   max_delta   std_dev    window
==============================================================================
/mavros/vision_pose/pose      30.0   0.02615     0.03826     0.001201   330   
/mavros/local_position/pose   30.0   0.027       0.04035     0.001645   330   

^C           topic              rate   min_delta   max_delta   std_dev    window
==============================================================================
/mavros/vision_pose/pose      30.0   0.02615     0.03826     0.001193   340   
/mavros/local_position/pose   30.0   0.027       0.04035     0.001646   340   

password123456@ubuntu:~$ 
