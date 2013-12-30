from pylab import *
import rosbag
import numpy

import movingAverage
import velocity

clicks_per_m = 15768.6
   
bagFile = "/home/jamie/Downloads/gps_test5/360.bag"
front_encoders = "/roboteq_front/encoders"
back_encoders = "/roboteq_back/encoders"
front_cmds = "/roboteq_front/cmd_vel"
back_cmds = "/roboteq_back/cmd_vel"
imu = "/imu/data"
gyros = "/imu/integrated_gyros_stamped"
bag = rosbag.Bag(bagFile)

front_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[front_encoders])]
back_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_encoders])]
front_cmd_msgs = [msg for topic,msg,t in bag.read_messages(topics=[front_cmds])]
back_cmd_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_cmds])]
imu_msgs = [msg for topic,msg,t in bag.read_messages(topics=[imu])]
imu_time = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in imu_msgs]
gyros_msgs = [msg for topic,msg,t in bag.read_messages(topics=[gyros])]
gyros_time = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in gyros_msgs]

imu_avg_time = movingAverage.exponentialMovingAverage(imu_time,alpha=0.5)
gyros_avg_time = movingAverage.movingAverage(gyros_time,1)

front_cmd_time = numpy.linspace(imu_time[0],imu_time[-1],len(front_cmd_msgs))
back_cmd_time = numpy.linspace(imu_time[0],imu_time[-1],len(back_cmd_msgs))

orig_cmd_z   = [t.angular.z for t in front_cmd_msgs]
scale_z      = [t.angular.z/2 for t in front_cmd_msgs]
imu_z        = [t.angular_velocity.z for t in imu_msgs]
gyro_z       = [t.vector.z for t in gyros_msgs]
imu_avg_z    = movingAverage.exponentialMovingAverage(imu_z,alpha=0.1)
gyro_avg_z   = movingAverage.exponentialMovingAverage(gyro_z,alpha=0.1)
gyro_avg_w   = velocity.getAngularImuVelocities(gyro_avg_z,imu_avg_time)

#p1,=plot(imu_time,imu_z,'-r')
p1,=plot(imu_avg_time,imu_avg_z,'-r')
p2,=plot(front_cmd_time,orig_cmd_z,'-b')
p3,=plot(back_cmd_time,scale_z,'-g')
p4,=plot(gyros_avg_time,gyro_avg_w,'-k')
p5,=plot(gyros_time,gyro_avg_z,':k')
xlabel("Time")
ylabel("rad/s")
title("Rotate in place experiment")
legend([p1,p2,p3,p4,p5],["imu","original cmd","scaled cmd", "angular integrated gyros", "orientation integrated gyros"])
show() 
