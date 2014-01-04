from pylab import *
import os
import rosbag
import numpy
import bisect
import math
import glob

import movingAverage
import velocity
import sync
import plot_velocity

clicks_per_m = 15768.6
bagFile = "/home/redblade/ROBOTEQDATA/ekfout.bag"
front_encoders = "/encoders_front"
back_encoders = "/encoders_back"
front_cmds = "/roboteq_front/cmd_vel_stamped"
back_cmds = "/roboteq_back/cmd_vel_stamped"
imu = "/imu/data"
gyros = "/imu/integrated_gyros_stamped"
gps = "/gps"
cmd_vels = "/cmd_vel"
odom = "/odom"
ekf = "/redblade_ekf/odom"
ekf2d = "/redblade_ekf/2d_pose"

bag = rosbag.Bag(bagFile)
threshold = 1.0
gyro_msgs   = [msg for topic,msg,t in bag.read_messages(topics=[gyros])]
ekf_2d_msgs = [msg for topic,msg,t in bag.read_messages(topics=[ekf2d])]

gyro_time   = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in gyro_msgs]
gyro_z      = [t.vector.z for t in gyro_msgs]
ekf_time    = numpy.linspace(gyro_time[0],gyro_time[-1],len(gyro_msgs))
ekf_z       = [t.theta for t in ekf_2d_msgs]
ekf_z,gyro_z = sync.matchInput(ekf_time,ekf_z,gyro_time,gyro_z,threshold)
print gyro_z
error = [0]*len(ekf_z)
for i in range(0,len(ekf_z)):
    # if ekf_z[i]-gyro_z[i]>2*math.pi:
    #     error[i] =ekf_z[i]-gyro_z[i]+2*math.pi
    # elif ekf_z[i]-gyro_z[i]>2*math.pi:
    #     error[i] =ekf_z[i]-gyro_z[i]-2*math.pi
    # else:
    #print ekf_z[i],gyro_z[i]
    error[i] = ekf_z[i]-gyro_z[i]

i = range(len(error))
plot(i,error)
show()
