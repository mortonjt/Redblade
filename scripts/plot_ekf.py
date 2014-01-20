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
#import plot_velocity

clicks_per_m = 15768.6
bagFile = "/home/jamie/DATA/EKF/ekf_processed.bag"
imu = "/imu/data"
gyros = "/imu/integrated_gyros_stamped"
gps = "/gps"
cmd_vels = "/cmd_vel"
odom = "/odom"
ekf = "/redblade_ekf/odom"
ekf2d = "/redblade_ekf/2d_pose"
front_encoders = "/encoders_front"
back_encoders  = "/encoders_back"
front_cmds     = "/roboteq_front/cmd_vel_stamped"
back_cmds      = "/roboteq_back/cmd_vel_stamped"
imu            = "/imu/data"
gps            = "/gps"
cmd_vels       = "/Arduino_RC"
odom       = "/odom"

bag = rosbag.Bag(bagFile)
threshold = 0.2
odom_msgs   = [msg for topic,msg,t in bag.read_messages(topics=[odom])]
gps_msgs   = [msg for topic,msg,t in bag.read_messages(topics=[gps])]
gyro_msgs   = [msg for topic,msg,t in bag.read_messages(topics=[gyros])]
ekf_2d_msgs = [msg for topic,msg,t in bag.read_messages(topics=[ekf2d])]
front_encoder_msgs= [msg for topic,msg,t in bag.read_messages(topics=[front_encoders])]
back_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_encoders])]

gyro_time   = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in gyro_msgs]
gyro_z      = [t.vector.z for t in gyro_msgs]
gps_time    = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in gps_msgs]
ekf_time    = numpy.linspace(gps_time[0],gps_time[-1],len(ekf_2d_msgs))
odom_time   = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in odom_msgs]
odom_vel    = [t.twist.twist.linear.x for t in odom_msgs]
odom_ang    = [t.twist.twist.angular.z for t in odom_msgs]
ekf_z       = [t.theta for t in ekf_2d_msgs]
ekf_x       = [t.x for t in ekf_2d_msgs]
ekf_y       = [t.y for t in ekf_2d_msgs]

ekf_z,gyro_z = sync.matchInput(ekf_time,ekf_z,gyro_time,gyro_z,threshold)
gps_x = [x.pose.pose.position.x for x in gps_msgs]
gps_y = [y.pose.pose.position.y for y in gps_msgs]

gps_time = [gps_time[i] for i in range(0,len(gps_time),2)]
gps_x    = [gps_x[i] for i in range(0,len(gps_x),2)]
gps_y    = [gps_y[i] for i in range(0,len(gps_y),2)]
gpsVels    = velocity.getLinearGPSVelocities(gps_x,gps_y,gps_time)

ekf_sync_x,gps_sync_x = sync.matchInput(ekf_time,ekf_x,gps_time,gps_x,threshold)
ekf_sync_y,gps_sync_y = sync.matchInput(ekf_time,ekf_y,gps_time,gps_y,threshold)
errorX = [gps_sync_x[i]-ekf_sync_x[i] for i in range(len(ekf_sync_x))]
errorY = [gps_sync_y[i]-ekf_sync_y[i] for i in range(len(ekf_sync_y))]
#print "\n".join(map(str,zip(gps_sync_x,ekf_sync_x)))
#print "\n".join(map(str,zip(time,ekf_time)))
#print "\n".join(map(str,gps_time))
font = {'weight' : 'bold',
        'size'   : 22}

matplotlib.rc('font', **font)

denied_x1 = gps_x[25:50]
denied_y1 = gps_y[25:50]
denied_x2 = gps_x[75:100]
denied_y2 = gps_y[75:100]
f1=figure(1)
title("Truth vs. EKF Position")
grid()

p3,=plot(denied_x1,denied_y1,'-c',linewidth=20.0)
plot(denied_x2[:-1],denied_y2[:-1],'-c',linewidth=20.0)
p1,=plot(gps_x[:-1],gps_y[:-1],'-ob',linewidth=6.0,markersize=10.0)
p2,=plot(ekf_x[1:],ekf_y[1:],'-or')
p4,=plot(gps_x[0],gps_y[0],'og',markersize=20.0)
p5,=plot(gps_x[-2],gps_y[-2],'ok',markersize=20.0)

legend([p1,p2,p3,p4,p5],["Truth position","EKF position","Denied GPS","Start","End"])
xlabel("Easting (m)")
ylabel("Northing (m)")
f1.show()

#distance_err = [math.sqrt(errorX[i]*errorX[i]+errorY[i]*errorY[i])]

# error = [0]*len(ekf_z)
# for i in range(0,len(ekf_z)):
#     if(abs(ekf_z[i]-gyro_z[i])<math.pi):
#         error[i] = ekf_z[i]-gyro_z[i]
#     else:
#         error[i] = ekf_z[i]-gyro_z[i]+2*math.pi
# t = range(len(error))

# f2=figure(2)
# plot(t,error)
# title("IMU error")
# f2.show()

# tX = range(len(errorX))
# f3=figure(3)
# plot(tX,errorX)
# title("X error")
# f3.show()

# tY = range(len(errorY))
# f4=figure(4)
# plot(tY,errorY)
# title("Y error")
# f4.show()

# f5=figure(5)
# title("X Position")
# p1,=plot(gps_time,gps_x,'ob')
# p2,=plot(ekf_time,ekf_x,'r')
# legend([p1,p2],["GPS Robot position","EKF position"])
# f5.show()

# f6=figure(6)
# title("Y Position")
# p1,=plot(gps_time,gps_y,'ob')
# p2,=plot(ekf_time,ekf_y,'r')
# legend([p1,p2],["GPS Robot position","EKF position"])
# f6.show()

# f7=figure(7)
# title("Velocity")
# p1,plot(gps_time[:-1],gpsVels,'ob')
# p2,plot(odom_time,odom_vel,'or')
# legend([p1,p2],["GPS velocities","Encoder Velocities"])
# f7.show()

# f8=figure(8)
# title("Angular Velocity")
# plot(odom_time,odom_ang,'or')
# f8.show()

# dT = [odom_time[i] - odom_time[i-1] for i in range(0,len(odom_time))]
# odom_ang_diff = [dT[i-1]*odom_ang[i] for i in range(1,len(odom_ang))]
# odom_ang_pos  = [odom_ang_diff[0]]*len(odom_ang_diff)
# for i in range(1,len(odom_ang_pos)):
#     odom_ang_pos[i] = odom_ang_diff[i]+odom_ang_pos[i-1]

# f9=figure(9)
# title("Angular Displacement")
# plot(odom_time[:len(odom_ang_pos)],odom_ang_pos,'or')
# plot(gyro_time[:len(gyro_z)],gyro_z,'ob')
# f9.show()

raw_input()
