from pylab import *
import rosbag
import numpy
import bisect
import math

import movingAverage

gpsBag = "/home/jamie/Documents/data/stereo_test/test2/calibrate.bag"
poleBag = "/home/jamie/Documents/data/stereo_test/test2/correctedPoints2.bag"

front_encoders = "/encoders_front"
back_encoders = "/encoders_back"
front_cmds = "/roboteq_front/cmd_vel_stamped"
back_cmds = "/roboteq_back/cmd_vel_stamped"
imu = "/imu/data"
gps = "/gps"
cmd_vels = "/cmd_vel"
pole_point = "/stereo_camera/pole"

bag = rosbag.Bag(gpsBag)
threshold = 0.1
gps_msgs          = [msg for topic,msg,t in bag.read_messages(topics=[gps])]

gps_x = [x.pose.pose.position.x for x in gps_msgs]
gps_y = [y.pose.pose.position.y for y in gps_msgs]
time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9 for t in gps_msgs]


#print min(time),max(time),len(time)
#print "Average GPS position",numpy.mean(gps_x[-50:]),numpy.mean(gps_y[-50:])

correct_x = gps_x[-50:]
correct_y = gps_y[-50:]

bag = rosbag.Bag(poleBag)
threshold = 0.1
gps_msgs           = [msg for topic,msg,t in bag.read_messages(topics=[gps])]
pole_msgs          = [msg for topic,msg,t in bag.read_messages(topics=[pole_point])]

pole_x = [x.point.x for x in pole_msgs]
pole_y = [y.point.y for y in pole_msgs]
time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9 for t in gps_msgs]

#print "GPS","\n".join(map(str,zip(correct_x,correct_y)))
print "Pole","\n".join(map(str,zip(pole_x,pole_y)))

print "X std dev",numpy.std(pole_x[1:])
print "Y std dev",numpy.std(pole_y[1:])

f1 = figure(1)
p1,=plot(pole_y,pole_x,'ob')
p2,=plot(correct_y,correct_x,'or')
ylabel("x")
xlabel("y")
title("Experiment positions")
legend([p1,p2],["Stereo vision estimates"," GPS estimates"])
f1.show() 



raw_input()
