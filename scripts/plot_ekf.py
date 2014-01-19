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
bagFile = "/home/jamie/DATA/EKF/ekf_processed.bag"
imu = "/imu/data"
gyros = "/imu/integrated_gyros_stamped"
gps = "/gps"
cmd_vels = "/cmd_vel"
odom = "/odom"
ekf = "/redblade_ekf/odom"
ekf2d = "/redblade_ekf/2d_pose"

bag = rosbag.Bag(bagFile)
threshold = 1.0
gps_msgs   = [msg for topic,msg,t in bag.read_messages(topics=[gps])]
gyro_msgs   = [msg for topic,msg,t in bag.read_messages(topics=[gyros])]
ekf_2d_msgs = [msg for topic,msg,t in bag.read_messages(topics=[ekf2d])]

gyro_time   = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in gyro_msgs]
gyro_z      = [t.vector.z for t in gyro_msgs]
ekf_time    = numpy.linspace(gyro_time[0],gyro_time[-1],len(ekf_2d_msgs))
gps_time    = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in gps_msgs]

ekf_z       = [t.theta for t in ekf_2d_msgs]
ekf_x       = [t.x for t in ekf_2d_msgs]
ekf_y       = [t.y for t in ekf_2d_msgs]

ekf_z,gyro_z = sync.matchInput(ekf_time,ekf_z,gyro_time,gyro_z,threshold)
gps_x = [x.pose.pose.position.x for x in gps_msgs]
gps_y = [y.pose.pose.position.y for y in gps_msgs]
time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9 for t in gps_msgs]
ekf_sync_x,gps_sync_x = sync.matchInput(ekf_time,ekf_x,gps_time,gps_x,threshold)
ekf_sync_y,gps_sync_y = sync.matchInput(ekf_time,ekf_y,gps_time,gps_y,threshold)
errorX = [gps_sync_x[i]-ekf_sync_x[i] for i in range(len(ekf_sync_x))]
errorY = [gps_sync_y[i]-ekf_sync_y[i] for i in range(len(ekf_sync_y))]


f1=figure(1)
title("GPS Position")
p1,=plot(gps_x,gps_y,'ob')
p2,=plot(ekf_x,ekf_y,'r')
# #legend([p1,p2,p3],["GPS Robot position"," Pole position (STEREO)","Correct position"])
f1.show()

error = [0]*len(ekf_z)
for i in range(0,len(ekf_z)):
    if(abs(ekf_z[i]-gyro_z[i])<math.pi):
        error[i] = ekf_z[i]-gyro_z[i]
    else:
        error[i] = ekf_z[i]-gyro_z[i]+2*math.pi
t = range(len(error))
f2=figure(2)
plot(t,error)
title("IMU error")
f2.show()

tX = range(len(errorX))
f3=figure(3)
plot(tX,errorX)
title("X error")
f3.show()

tY = range(len(errorY))
f4=figure(4)
plot(tY,errorY)
title("Y error")
f4.show()


raw_input()
