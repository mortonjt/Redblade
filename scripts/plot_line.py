from pylab import *
import rosbag
import numpy

import movingAverage
import velocity

clicks_per_m = 15768.6

bagFile = "/home/jamie/Downloads/gps_test5/line.bag"
front_encoders = "/roboteq_front/encoders"
back_encoders = "/roboteq_back/encoders"
front_cmds = "/roboteq_front/cmd_vel"
back_cmds = "/roboteq_back/cmd_vel"
imu = "/imu/data"
gps = "/gps"
bag = rosbag.Bag(bagFile)

front_encoder_msgs= [msg for topic,msg,t in bag.read_messages(topics=[front_encoders])]
back_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_encoders])]
front_cmd_msgs    = [msg for topic,msg,t in bag.read_messages(topics=[front_cmds])]
back_cmd_msgs     = [msg for topic,msg,t in bag.read_messages(topics=[back_cmds])]
imu_msgs          = [msg for topic,msg,t in bag.read_messages(topics=[imu])]
gps_msgs          = [msg for topic,msg,t in bag.read_messages(topics=[gps])]

gps_x = movingAverage.exponentialMovingAverage(\
    [x.pose.pose.position.x for x in gps_msgs],alpha=0.5)
gps_y = movingAverage.exponentialMovingAverage(\
    [y.pose.pose.position.y for y in gps_msgs],alpha=0.5)
time  = movingAverage.exponentialMovingAverage(\
    [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9 for t in gps_msgs],alpha=0.5)


gpsTime    = movingAverage.movingAverage(\
    [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
     for t in gps_msgs],1)
gpsVels    = velocity.getLinearGPSVelocities(gps_x,gps_y,time)

front_time = movingAverage.movingAverage(\
    [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
     for t in front_encoder_msgs],1)
back_time  = movingAverage.movingAverage(\
    [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
     for t in back_encoder_msgs],1)

frontVels,backVels = velocity.getLinearEncoderVelocities(\
    front_encoder_msgs,back_encoder_msgs)
# frontVels = movingAverage.exponentialMovingAverage(
#     frontVels,alpha=0.5)
# backVels = movingAverage.exponentialMovingAverage(
#     backVels,alpha=0.5)

front_cmd_time = numpy.linspace(gpsTime[0],gpsTime[-1],len(front_cmd_msgs))    
front_cmd_vels = [t.linear.x for t in front_cmd_msgs]
p1,=plot(front_time,frontVels,'-g')
p2,=plot(back_time,backVels,'-b')
p3,=plot(gpsTime,gpsVels,'-ok')
p4,=plot(front_cmd_time,front_cmd_vels,'-r')
xlabel("Time")
ylabel("m/s")
title("Straight line experiment")
legend([p1,p2,p3,p4],["front encoder velocity",
                      "back encoder velocity",
                      "gps velocity",
                      "cmd velocity"])
show() 

