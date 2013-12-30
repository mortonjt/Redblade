# from pylab import *

# import rosbag
# import bisect

# clicks_per_m = 15768.6;

# def getLinearGPSVelocity(paired_msgs):
#     prev_msg,cur_msg = paired_msgs
#     prev_x,prev_y,prev_time = prev_msg
#     cur_x,cur_y,cur_time = cur_msg
#     dTime = float(cur_time-prev_time)
#     distance = sqrt( (cur_x-prev_x)*(cur_x-prev_x)+\
#                      (cur_y-prev_y)*(cur_y-prev_y) )
#     return distance/dTime,cur_time

# def getLinearEncoderVelocity(paired_msgs):
#     prev_msg,cur_msg = paired_msgs
#     prev_front,prev_back = prev_msg
#     cur_front,cur_back = cur_msg
#     dTime = (cur_front.encoders.time_delta+cur_back.encoders.time_delta)/2.0
#     prev_left_pos  = (prev_front.encoders.left_wheel+\
#                       prev_back.encoders.left_wheel)/(2.0*clicks_per_m)
#     prev_right_pos = (prev_front.encoders.right_wheel+\
#                       prev_back.encoders.right_wheel)/(2.0*clicks_per_m)
#     cur_left_pos   = (cur_front.encoders.left_wheel+\
#                       cur_back.encoders.left_wheel)/(2.0*clicks_per_m)
#     cur_right_pos  = (cur_front.encoders.right_wheel+\
#                       cur_back.encoders.right_wheel)/(2.0*clicks_per_m)
#     left_vel = (cur_left_pos-prev_left_pos)/dTime
#     right_vel = (cur_right_pos-prev_right_pos)/dTime
#     time = cur_front.header.stamp.secs+cur_front.header.stamp.nsecs/10**9
#     return (left_vel+right_vel)/2.0,time

# def movingAverage(gps_windows):
#     x_avg = sum([msg.pose.pose.position.x for msg in gps_windows])/len(gps_windows)
#     y_avg = sum([msg.pose.pose.position.y for msg in gps_windows])/len(gps_windows)
#     time = sum([msg.header.stamp.secs+msg.header.stamp.nsecs/10**9
#                 for msg in gps_windows])/float(len(gps_windows))
#     return x_avg,y_avg,time

# def weightedMovingAverage(gps_windows):
#     n = len(gps_windows)
#     x_avg = sum([gps_windows[i].pose.pose.position.x*(n-i)
#                  for i in range(0,len(gps_windows))])/sum(range(0,n))
#     y_avg = sum([gps_windows[i].pose.pose.position.y*(n-i)
#                  for i in range(0,len(gps_windows))])/sum(range(0,n))
#     time = sum([msg.header.stamp.secs+msg.header.stamp.nsecs/10**9
#                 for msg in gps_windows])/n
#     return x_avg,y_avg,time

# def exponentialMovingAverage(gps_msgs,alpha=0.5):
#     Sx = [gps_msgs[0].pose.pose.position.x]*(len(gps_msgs))
#     Sy = [gps_msgs[0].pose.pose.position.y]*(len(gps_msgs))
#     time = [msg.header.stamp.secs+msg.header.stamp.nsecs/10.0**9 for msg in gps_msgs]
#     for i in range(1,len(gps_msgs)):
#         Sx[i] = alpha*gps_msgs[i-1].pose.pose.position.x+\
#                 (1-alpha)*Sx[i-1]
#         Sy[i] = alpha*gps_msgs[i-1].pose.pose.position.y+\
#                 (1-alpha)*Sy[i-1]
#     return zip(Sx,Sy,time)

# """
# Filter encoders based on GPS measurements
# Only accept the closest encoder reads to GPS measurements in with respect to time
# """
# def filterEncoders(encoders,gps):
#     encoderVels,encoder_times = zip(*encoders)
#     gpsVels,gps_times = zip(*gps)
#     gpsIndexs = range(0,len(gps_times))
#     encoderIndexs = []
#     times = set(encoder_times)
#     for i in gpsIndexs:
#         tmp = list(times)
#         index=bisect.bisect_left(tmp,gps_times[i])
#         print len(tmp),index
#         times.remove(tmp[index])
#         encoderIndexs.append(index)
#     #gpsIndexs,encoderIndexs = gpsDict.keys(),gpsDict.values()
#     print gpsDict
#     #print max(encoderIndexs),len(encoderIndexs),len(gpsIndexs)
#     gpsVels = [gpsVels[i] for i in gpsIndexs]
#     encoderVels = [encoderVels[i] for i in encoderIndexs]
#     return gpsVels,encoderVels

# bagFile = "/home/jamie/Downloads/gps_test3/2013-12-28-16-58-58.bag"
# front_encoders = "/roboteq_front/encoders"
# back_encoders = "/roboteq_back/encoders"
# gps = "/gps"
# bag = rosbag.Bag(bagFile)

# linVels = []

# front_msgs = [msg for topic,msg,t in bag.read_messages(topics=[front_encoders])]
# back_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_encoders])]
# gps_msgs = [msg for topic,msg,t in bag.read_messages(topics=[gps])]

# ##Speeds from encoder readings
# roboteq_msgs = zip(front_msgs,back_msgs)
# paired_msgs = zip(roboteq_msgs[:-1],roboteq_msgs[1:]) #previous paired with current msg
# encoderPairs = map( getLinearEncoderVelocity, paired_msgs ) #Has both velocity and time

# ##Speeds from gps readings
# # gps_windows = zip(gps_msgs[0:-5],
# #                   gps_msgs[1:-4],
# #                   gps_msgs[2:-3],
# #                   gps_msgs[3:-2],
# #                   gps_msgs[4:-1],
# #                   gps_msgs[5:])
# # gps_avgs = map(weightedMovingAverage,gps_windows)
# gps_avgs = exponentialMovingAverage(gps_msgs)
# paired_msgs = zip(gps_avgs[:-1],gps_avgs[1:])
# gpsPairs = map(getLinearGPSVelocity,paired_msgs) #Has both velocity and time

# gpsVels,gpsTime = zip(*gpsPairs)
# encoderVels,gpsVels = filterEncoders(encoderPairs,gpsPairs)


# f = figure(1)
# plot(encoderVels,gpsVels,'ob')
# title('Encoder vs GPS velocity')
# xlabel('Encoder Velocities')
# ylabel('GPS Velocities')
# f.show()

# g = figure(2)
# diff = [vel[1]-vel[0] for vel in zip(gpsVels,encoderVels)]
# plot(encoderVels,diff,'ob')
# title('Encoder vs Error')
# xlabel('Encoder Velocities')
# ylabel('Velocity Error')
# g.show()

# t = figure(3)
# plot(gpsTime,gpsVels,'ob')
# title('GPS velocity vs Time')
# xlabel('Time')
# ylabel('GPS Velocities')
# t.show()

# raw_input()



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

