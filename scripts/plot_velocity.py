from pylab import *

import rosbag

clicks_per_m = 15768.6;

def getLinearGPSVelocity(paired_msgs):
    prev_msg,cur_msg = paired_msgs
    prev_x,prev_y,prev_time = prev_msg
    cur_x,cur_y,cur_time = cur_msg
    dTime = float(cur_time-prev_time)
    distance = sqrt( (cur_x-prev_x)*(cur_x-prev_x)+\
                     (cur_y-prev_y)*(cur_y-prev_y) )
    return distance/dTime,int(cur_time)

def getLinearEncoderVelocity(paired_msgs):
    prev_msg,cur_msg = paired_msgs
    prev_front,prev_back = prev_msg
    cur_front,cur_back = cur_msg
    dTime = (cur_front.encoders.time_delta+cur_back.encoders.time_delta)/2.0
    prev_left_pos  = (prev_front.encoders.left_wheel+\
                      prev_back.encoders.left_wheel)/(2.0*clicks_per_m)
    prev_right_pos = (prev_front.encoders.right_wheel+\
                      prev_back.encoders.right_wheel)/(2.0*clicks_per_m)
    cur_left_pos   = (cur_front.encoders.left_wheel+\
                      cur_back.encoders.left_wheel)/(2.0*clicks_per_m)
    cur_right_pos  = (cur_front.encoders.right_wheel+\
                      cur_back.encoders.right_wheel)/(2.0*clicks_per_m)
    left_vel = (cur_left_pos-prev_left_pos)/dTime
    right_vel = (cur_right_pos-prev_right_pos)/dTime
    return (left_vel+right_vel)/2.0,cur_front.header.stamp.secs

def movingAverage(gps_windows):
    x_avg = sum([msg.pose.pose.position.x for msg in gps_windows])/len(gps_windows)
    y_avg = sum([msg.pose.pose.position.y for msg in gps_windows])/len(gps_windows)
    time = sum([msg.header.stamp.secs+msg.header.stamp.nsecs/10**9
                for msg in gps_windows])/float(len(gps_windows))
    return x_avg,y_avg,time
    
    
bagFile = "/home/jamie/Downloads/gps_test3/2013-12-28-16-58-58.bag"
front_encoders = "/roboteq_front/encoders"
back_encoders = "/roboteq_back/encoders"
gps = "/gps"
bag = rosbag.Bag(bagFile)

linVels = []

front_msgs = [msg for topic,msg,t in bag.read_messages(topics=[front_encoders])]
back_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_encoders])]
gps_msgs = [msg for topic,msg,t in bag.read_messages(topics=[gps])]

##Speeds from encoder readings
roboteq_msgs = zip(front_msgs,back_msgs)
paired_msgs = zip(roboteq_msgs[:-1],roboteq_msgs[1:]) #previous paired with current msg
encoderVels = map( getLinearEncoderVelocity, paired_msgs )

##Speeds from gps readings
gps_windows = zip(gps_msgs[0:-5],
                  gps_msgs[1:-4],
                  gps_msgs[2:-3],
                  gps_msgs[3:-2],
                  gps_msgs[4:-1],
                  gps_msgs[5:])

gps_avgs = map(movingAverage,gps_windows)
paired_msgs = zip(gps_avgs[:-1],gps_avgs[1:])
gpsVels = map(getLinearGPSVelocity,paired_msgs)
#Get only the first velocity seen at each second
gpsVelsDict = {t[1]:t[0] for t in gpsVels }
encoderVelsDict = {t[1]:t[0] for t in encoderVels }

gpsVels = gpsVelsDict.values()
encoderVels = encoderVelsDict.values()[:-1]

f = figure(1)
print len(encoderVels),len(gpsVels)
plot(encoderVels,gpsVels,'ob')
title('Encoder vs GPS velocity')
xlabel('Encoder Velocities')
ylabel('GPS Velocities')
f.show()

g = figure(2)
diff = [vel[1]-vel[0] for vel in zip(gpsVels,encoderVels)]
plot(encoderVels,diff,'ob')
title('Encoder vs Error')
xlabel('Encoder Velocities')
ylabel('Velocity Error')
g.show()

raw_input()
