from pylab import *
import rosbag
import numpy
import bisect
import math

import movingAverage

gpsBag = "/home/jamie/Documents/data/stereo_test/calibrate_gps.bag"
poleBag = "/home/jamie/Documents/data/stereo_test/pole.bag"

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
front_encoder_msgs= [msg for topic,msg,t in bag.read_messages(topics=[front_encoders])]
back_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_encoders])]
front_cmd_msgs    = [msg for topic,msg,t in bag.read_messages(topics=[front_cmds])]
back_cmd_msgs     = [msg for topic,msg,t in bag.read_messages(topics=[back_cmds])]
imu_msgs          = [msg for topic,msg,t in bag.read_messages(topics=[imu])]
gps_msgs          = [msg for topic,msg,t in bag.read_messages(topics=[gps])]
cmd_vel_msgs      = [msg for topic,msg,t in bag.read_messages(topics=[cmd_vels])]

gps_x = [x.pose.pose.position.x for x in gps_msgs]
gps_y = [y.pose.pose.position.y for y in gps_msgs]
time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9 for t in gps_msgs]

#print min(time),max(time),len(time)
#print "Average GPS position",numpy.mean(gps_x[-50:]),numpy.mean(gps_y[-50:])

correct_x = gps_x[-50:]
correct_y = gps_y[-50:]

bag = rosbag.Bag(poleBag)
threshold = 0.1
front_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[front_encoders])]
back_encoder_msgs  = [msg for topic,msg,t in bag.read_messages(topics=[back_encoders])]
front_cmd_msgs     = [msg for topic,msg,t in bag.read_messages(topics=[front_cmds])]
back_cmd_msgs      = [msg for topic,msg,t in bag.read_messages(topics=[back_cmds])]
imu_msgs           = [msg for topic,msg,t in bag.read_messages(topics=[imu])]
gps_msgs           = [msg for topic,msg,t in bag.read_messages(topics=[gps])]
cmd_vel_msgs       = [msg for topic,msg,t in bag.read_messages(topics=[cmd_vels])]
pole_msgs          = [msg for topic,msg,t in bag.read_messages(topics=[pole_point])]

pole_x = [x.x for x in pole_msgs]
pole_y = [y.y for y in pole_msgs]
time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9 for t in gps_msgs]

#print "GPS","\n".join(map(str,zip(correct_x,correct_y)))
#print "Pole","\n".join(map(str,zip(pole_x,pole_y)))

f1 = figure(1)
plot(pole_x,pole_y,'ob')
plot(correct_x,correct_y,'or')
xlabel("x")
ylabel("y")
title("Experiment positions")
f1.show() 

raw_input()
