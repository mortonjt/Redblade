from pylab import *
import rosbag
import numpy

import movingAverage
import velocity

clicks_per_m = 15768.6
bagFile = "/home/redblade/ROBOTEQDATA/twist_converter_test7/360.bag"
front_encoders = "/encoders_front"
back_encoders = "encoders_back"
front_cmds = "/roboteq_front/cmd_vel_stamped"
back_cmds = "/roboteq_back/cmd_vel_stamped"
imu = "/imu/data"
gyros = "/imu/integrated_gyros_stamped"
cmd_vels = "/Arduino_RC"
bag = rosbag.Bag(bagFile)

front_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[front_encoders])]
back_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_encoders])]
front_cmd_msgs = [msg for topic,msg,t in bag.read_messages(topics=[front_cmds])]
back_cmd_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_cmds])]
imu_msgs = [msg for topic,msg,t in bag.read_messages(topics=[imu])]
imu_time = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in imu_msgs]
gyros_msgs = [msg for topic,msg,t in bag.read_messages(topics=[gyros])]
gyros_time = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in gyros_msgs]
#imu_avg_time = movingAverage.exponentialMovingAverage(imu_time,alpha=0.5)
imu_avg_time = imu_time
gyros_avg_time = movingAverage.movingAverage(gyros_time,1)
cmd_vel_msgs = [msg for topic,msg,t in bag.read_messages(topics=[cmd_vels])]
cmd_vels_time = numpy.linspace(imu_time[0],imu_time[-1],len(cmd_vel_msgs))


front_cmd_time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
     for t in front_cmd_msgs]
back_cmd_time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
     for t in back_cmd_msgs]


# front_cmd_time = numpy.linspace(imu_time[0],imu_time[-1],len(front_cmd_msgs))
# back_cmd_time = numpy.linspace(imu_time[0],imu_time[-1],len(back_cmd_msgs))

orig_cmd_z   = [t.twist.angular.z for t in front_cmd_msgs]
imu_z        = [t.angular_velocity.z for t in imu_msgs]
gyro_z       = [t.vector.z for t in gyros_msgs]
imu_avg_z    = movingAverage.exponentialMovingAverage(imu_z,alpha=0.1)
gyro_avg_z   = movingAverage.exponentialMovingAverage(gyro_z,alpha=0.1)
gyro_avg_w   = velocity.getAngularImuVelocities(gyro_avg_z,imu_avg_time)
cmd_vels_z   = [t.angular.z for t in cmd_vel_msgs]

scaled = [x[0]/x[1] for x in zip(imu_z,cmd_vels_z) if x[1]>0.1]
scaling_factor = sum(scaled)/float(len(scaled))
print "scaling factor",scaling_factor

p1,=plot(imu_time,imu_z,'-r')
#p1,=plot(imu_avg_time,imu_avg_z,'-r')
p2,=plot(front_cmd_time,orig_cmd_z,'-b')
p3,=plot(cmd_vels_time,cmd_vels_z,'-k')
# p4,=plot(gyros_avg_time,gyro_avg_w,'-k')
# p5,=plot(gyros_time,gyro_avg_z,':k')
xlabel("Time")
ylabel("rad/s")
title("Rotate in place experiment")
legend([p1,p2,p3],["imu","roboteq cmd vel","arduino cmd vel"])
show() 
