
from pylab import *
import rosbag
import numpy
import bisect

import movingAverage
import velocity


clicks_per_m = 15768.6

def filterEncoders(gpsTimes,encoderTimes,encoderVels,threshold):
    filteredVels,filteredTimes = [],[]
    gpsTimes.sort()
    for i in xrange(0,len(encoderTimes)):
        j = bisect.bisect_left(gpsTimes,encoderTimes[i])
        if j<len(gpsTimes) and abs(gpsTimes[j]-encoderTimes[i]) < threshold:
            filteredVels.append(encoderVels[i])
            filteredTimes.append(encoderTimes[i])
    return filteredVels,filteredTimes


bagFile = "/home/jamie/Downloads/gps_test4/square.bag"
front_encoders = "/roboteq_front/encoders"
back_encoders = "/roboteq_back/encoders"
front_cmds = "/roboteq_front/cmd_vel_stamped"
back_cmds = "/roboteq_back/cmd_vel_stamped"
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

gpsTimes    = movingAverage.movingAverage(\
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

front_cmd_time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
     for t in front_cmd_msgs]
front_cmd_vels = [t.twist.linear.x for t in front_cmd_msgs]
#Preliminary analysis
f1 = figure(1)
plot(gps_x,gps_y,'ob')
xlabel("x")
ylabel("y")
title("Square experiment positions")
f1.show() 


f2 = figure(2)
p1,=plot(front_time,frontVels,'-g')
p2,=plot(back_time,backVels,'-b')
p3,=plot(gpsTimes,gpsVels,'-k')
p4,=plot(front_cmd_time,front_cmd_vels,':r')
xlabel("Time")
ylabel("m/s")
title("Square experiment velocities")
legend([p1,p2,p3,p4],["front encoder velocity",
                      "back encoder velocity",
                      "gps velocity",
                      "cmd velocity"])
f2.show() 


#Error Analysis
threshold = 0.01
frontVels,frontTimes = filterEncoders(gpsTimes,front_time,frontVels,threshold)
backVels,backTimes   = filterEncoders(gpsTimes,back_time,backVels,threshold)
gpsTimeBins          = [gpsTimes[i]-threshold
                        for i in xrange(0,len(gpsTimes))]
frontIndexes         = numpy.digitize(frontTimes,gpsTimeBins)[:-1]
backIndexes          = numpy.digitize(backTimes,gpsTimeBins)[:-1]

frontVels    = [frontVels[i] for i in xrange(0,len(frontIndexes))]
backVels     = [backVels[i]  for i in xrange(0,len(backIndexes))]
gpsFrontVels = [gpsVels[frontIndexes[i]] for i in xrange(0,len(frontIndexes))]
gpsBackVels  = [gpsVels[backIndexes[i]]  for i in xrange(0,len(backIndexes))]
frontError   = [gpsVels[i]-frontVels[i]  for i in xrange(0,len(frontVels))]
backError    = [gpsVels[i]-backVels[i]   for i in xrange(0,len(backVels))]

f3 = figure(3)
plot(frontVels,gpsFrontVels,'ob')
plot(backVels,gpsBackVels,'ob')
xlabel("Encoder Velocities (m/s)")
ylabel("GPS Velocities (m/s)")
title("Square experiment gps encoder velocities")
f3.show() 

f4 = figure(4)
p1,=plot(frontVels,frontError,'.')
p2,=plot(backVels,backError,'.')
p3,=plot(gpsFrontVels,frontError,'.')
p4,=plot(gpsBackVels,backError,'.')
xlabel("Velocity (m/s)")
ylabel("Error (m/s)")
title("Square experiment gps encoder error")
legend([p1,p2,p3,p4],["front encoder velocity",
                      "back encoder velocity",
                      "front gps velocity",
                      "back gps velocity"])
f4.show() 
raw_input()
