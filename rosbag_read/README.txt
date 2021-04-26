Author: Hadi.Hajieghrary@Gmail.Com
LinkedIn: https://www.linkedin.com/in/hadihajieghrary/

This code could be used to read the messages inside the ROSbag. The code is implemented for nav_msgs/Odometry and sensor_msgs::Imu for example. The function "writeToFile" can be overloaded for other message types.

The messages of the topics of these types. if there exist any, will be written into the *.dat file. The name(s) of the file(s) will reflect the name of the topics (By default, "/" in the name of the topic will be replaced with "|" in the name of the file.) 

----

Usage:
>> cd-to-catkin-workspace
>>catkin_make
>>source devel/setup.bash
>>rosrun rosbag_read rosbag_read address-of-the-bag-file.bag

