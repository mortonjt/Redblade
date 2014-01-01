TOPICS=`rostopic list | tr "\n" " "`
#rosbag record /Arduino_RC /arduino_in /arduino_out /cmd_vel /diagnostics /encoders_back /encoders_front /imu/data /imu/integrated_gyros /imu/integrated_gyros_stamped /imu/is_calibrated /imu/orientation /roboteq_back/cmd_vel /roboteq_back/cmd_vel_stamped /roboteq_front/cmd_vel /roboteq_front/cmd_vel_stamped /rosout /rosout_agg
rosbag record $TOPICS