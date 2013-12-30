from pylab import *
import rosbag
import numpy

import movingAverage

clicks_per_m = 15768.6
   
bagFile = "/home/jamie/Downloads/gps_test5/360.bag"
front_encoders = "/roboteq_front/encoders"
back_encoders = "/roboteq_back/encoders"
front_cmds = "/roboteq_front/cmd_vel"
back_cmds = "/roboteq_back/cmd_vel"
imu = "/imu/data"
bag = rosbag.Bag(bagFile)

front_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[front_encoders])]
back_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_encoders])]
front_cmd_msgs = [msg for topic,msg,t in bag.read_messages(topics=[front_cmds])]
back_cmd_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_cmds])]
imu_msgs = [msg for topic,msg,t in bag.read_messages(topics=[imu])]
order = 20
imu_time = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in imu_msgs]
imu_avg_time = imu_time

front_cmd_time = numpy.linspace(imu_time[0],imu_time[-1],len(front_cmd_msgs))
back_cmd_time = numpy.linspace(imu_time[0],imu_time[-1],len(back_cmd_msgs))

orig_cmd_z        = [t.angular.z for t in front_cmd_msgs]
scale_z           = [t.angular.z/2 for t in front_cmd_msgs]
imu_z             = [t.angular_velocity.z for t in imu_msgs]
#imu_avg_z         = movingAverage.movingAverage(imu_z,order)
imu_avg_z = imu_z
imu_avg_z         = movingAverage.exponentialMovingAverage(imu_z,alpha=0.1)

#p1,=plot(imu_time,imu_z,'-r')
p1,=plot(imu_avg_time,imu_avg_z,'-r')
p2,=plot(front_cmd_time,orig_cmd_z,'-b')
p3,=plot(back_cmd_time,scale_z,'-g')
xlabel("Time")
ylabel("rad/s")
title("Rotate in place experiment")
legend([p1,p2,p3],["imu","original cmd","scaled cmd"])
show() 
