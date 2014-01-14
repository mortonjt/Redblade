from pylab import *
import rosbag
import numpy
import bisect
import math

import movingAverage

gpsBag = "/home/jamie/Documents/data/test4/pole.bag"
poleBag = "/home/jamie/Documents/data/test4/vision_processed.bag"
front_encoders = "/encoders_front"
back_encoders = "/encoders_back"
front_cmds = "/roboteq_front/cmd_vel_stamped"
back_cmds = "/roboteq_back/cmd_vel_stamped"
imu = "/imu/data"
gps = "/gps"
cmd_vels = "/cmd_vel"
lidar_pole = "/lidar/pole"
stereo_pole = "/stereo_camera/pole"

bag = rosbag.Bag(gpsBag)
threshold = 0.1
gps_msgs          = [msg for topic,msg,t in bag.read_messages(topics=[gps])]

gps_x = [x.pose.pose.position.x for x in gps_msgs]
gps_y = [y.pose.pose.position.y for y in gps_msgs]
time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9 for t in gps_msgs]


#print min(time),max(time),len(time)
#print "Average GPS position",numpy.mean(gps_x[-50:]),numpy.mean(gps_y[-50:])

correct_x = gps_x
correct_y = gps_y

bag = rosbag.Bag(poleBag)
threshold = 0.1
gps_msgs           = [msg for topic,msg,t in bag.read_messages(topics=[gps])]
lidar_msgs    = [msg for topic,msg,t in bag.read_messages(topics=[lidar_pole])]
stereo_msgs   = [msg for topic,msg,t in bag.read_messages(topics=[stereo_pole])]

lidar_pole_x = [x.point.x for x in lidar_msgs]
lidar_pole_y = [y.point.y for y in lidar_msgs]
stereo_pole_x = [x.point.x for x in stereo_msgs]
stereo_pole_y = [y.point.y for y in stereo_msgs]
time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9 for t in gps_msgs]

print "lidar X std dev",numpy.std(lidar_pole_x[1:])
print "lidar Y std dev",numpy.std(lidar_pole_y[1:])

f1 = figure(1)
p1,=plot(lidar_pole_y,lidar_pole_x,'ob')
p2,=plot(correct_y,correct_x,'or')
ylabel("x")
xlabel("y")
title("Experiment positions")
legend([p1,p2],["Lidar estimates"," GPS estimates"])
f1.show() 

print "stereo X std dev",numpy.std(stereo_pole_x[1:])
print "stereo Y std dev",numpy.std(stereo_pole_y[1:])

f2 = figure(2)
p1,=plot(stereo_pole_y,stereo_pole_x,'ob')
p2,=plot(correct_y,correct_x,'or')
ylabel("x")
xlabel("y")
title("Experiment positions")
legend([p1,p2],["Stereo camera estimates"," GPS estimates"])
f2.show() 


raw_input()
