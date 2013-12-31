from pylab import *
import rosbag
import numpy
import bisect
import math

import movingAverage
import velocity
import sync

clicks_per_m = 15768.6

def getAngularVels(bagFile,
                   front_encoders, 
                   back_encoders,  
                   front_cmds,      
                   back_cmds,      
                   imu,            
                   gps,            
                   cmd_vels):

    bag = rosbag.Bag(bagFile)
    threshold = 0.1
    front_encoder_msgs= [msg for topic,msg,t in bag.read_messages(topics=[front_encoders])]
    back_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_encoders])]
    front_cmd_msgs    = [msg for topic,msg,t in bag.read_messages(topics=[front_cmds])]
    back_cmd_msgs     = [msg for topic,msg,t in bag.read_messages(topics=[back_cmds])]
    imu_msgs          = [msg for topic,msg,t in bag.read_messages(topics=[imu])]
    gps_msgs          = [msg for topic,msg,t in bag.read_messages(topics=[gps])]
    cmd_vel_msgs      = [msg for topic,msg,t in bag.read_messages(topics=[cmd_vels])]

    imu_time   = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in imu_msgs]
    imu_z      = [t.angular_velocity.z for t in imu_msgs]
    cmd_vels_time = numpy.linspace(imu_time[0],imu_time[-1],len(cmd_vel_msgs))
    cmd_vels_z     = [t.angular.z for t in cmd_vel_msgs]
    cmdVels,ImuVels = sync.matchInput(cmd_vels_time,cmd_vels_z,imu_time,imu_z,threshold)
    return cmdVels,ImuVels

def plot(bagFile,
         front_encoders, 
         back_encoders,  
         front_cmds,      
         back_cmds,      
         imu,            
         gps,            
         cmd_vels): 
    bag = rosbag.Bag(bagFile)

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

    # gps_x = movingAverage.exponentialMovingAverage(\
    #     [x.pose.pose.position.x for x in gps_msgs],alpha=0.5)
    # gps_y = movingAverage.exponentialMovingAverage(\
    #     [y.pose.pose.position.y for y in gps_msgs],alpha=0.5)
    # time  = movingAverage.exponentialMovingAverage(\
    #     [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9 for t in gps_msgs],alpha=0.5)

    gpsTimes    = movingAverage.movingAverage(\
        [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
         for t in gps_msgs],1)

    # gpsVels    = movingAverage.exponentialMovingAverage(\
    #     velocity.getLinearGPSVelocities(gps_x,gps_y,time),alpha=0.5)

    gpsVels    = velocity.getLinearGPSVelocities(gps_x,gps_y,time)


    imu_time   = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in imu_msgs]
    imu_z      = [t.angular_velocity.z for t in imu_msgs]
    print "IMU velocity mean",numpy.mean(imu_z)
    roboteq_cmd_z   = [t.twist.angular.z for t in front_cmd_msgs]

    front_time = movingAverage.movingAverage(\
        [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
         for t in front_encoder_msgs],1)
    back_time  = movingAverage.movingAverage(\
        [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
         for t in back_encoder_msgs],1)

    frontVels,backVels = velocity.getLinearEncoderVelocities(\
        front_encoder_msgs,back_encoder_msgs)


    cmd_vels_time = numpy.linspace(imu_time[0],imu_time[-1],len(cmd_vel_msgs))
    cmd_vels_time_ = cmd_vels_time

    # frontVels = movingAverage.exponentialMovingAverage(
    #     frontVels,alpha=0.5)
    # backVels = movingAverage.exponentialMovingAverage(
    #     backVels,alpha=0.5)

    front_cmd_time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
         for t in front_cmd_msgs]
    front_cmd_vels = [t.twist.linear.x for t in front_cmd_msgs]

    # frontVels      = [-x for x in frontVels]
    # backVels       = [-x for x in backVels]
    frontVels_ = frontVels
    backVels_  = backVels
    #front_cmd_vels = [-x for x in front_cmd_vels]
    cmd_vels_z     = [t.angular.z for t in cmd_vel_msgs]
    cmd_vels_z_    = cmd_vels_z
    #Preliminary analysis
    f1 = figure(1)
    plot(gps_x,gps_y,'ob')
    xlabel("x")
    ylabel("y")
    title("Experiment positions")
    f1.show() 


    f2 = figure(2)
    p1,=plot(front_time,frontVels,'-g')
    p2,=plot(back_time,backVels,'-b')
    p3,=plot(gpsTimes,gpsVels,'-k')
    p4,=plot(front_cmd_time,front_cmd_vels,':r')
    xlabel("Time")
    ylabel("m/s")
    title("Experiment velocities")
    legend([p1,p2,p3,p4],["front encoder velocity",
                          "back encoder velocity",
                          "gps velocity",
                          "cmd velocity"])
    f2.show() 

    #Linear velocity error analysis
    threshold = 0.05
    gpsFrontVels,frontVels = sync.matchInput(gpsTimes,gpsVels,front_time,frontVels,threshold)
    gpsBackVels,backVels   = sync.matchInput(gpsTimes,gpsVels,back_time,backVels,threshold)

    frontError   = [gpsFrontVels[i]-frontVels[i]  for i in xrange(0,len(frontVels))]
    backError    = [gpsBackVels[i]-backVels[i]    for i in xrange(0,len(backVels))]

    print "Encoder mean",numpy.mean(frontError)
    print "Encoder std dev",numpy.std(frontError)

    f3 = figure(3)
    plot(frontVels,gpsFrontVels,'ob')
    plot(backVels,gpsBackVels,'ob')
    xlabel("Encoder Velocities (m/s)")
    ylabel("GPS Velocities (m/s)")
    title("Experiment gps encoder velocities")
    f3.show() 

    f4 = figure(4)
    p1,=plot(frontVels,frontError,'.')
    p2,=plot(backVels,backError,'.')
    p3,=plot(gpsFrontVels,frontError,'.')
    p4,=plot(gpsBackVels,backError,'.')
    xlabel("Velocity (m/s)")
    ylabel("Error (m/s)")
    title("Experiment gps encoder error")
    legend([p1,p2,p3,p4],["front encoder velocity",
                          "back encoder velocity",
                          "front gps velocity",
                          "back gps velocity"])
    f4.show() 

    threshold = 0.01

    frontVels,cmdVels = sync.matchInput(front_time,frontVels_,front_cmd_time,front_cmd_vels,threshold)

    f5 = figure(5)
    plot(cmdVels,frontVels,'ob')
    xlabel("Command Velocities (m/s)")
    ylabel("Encoder Velocities (m/s)")
    title("Experiment command encoder velocities")
    f5.show() 


    f6 = figure(6)
    p1,=plot(imu_time,imu_z,'-r')
    p2,=plot(front_cmd_time,roboteq_cmd_z,'-b')
    p3,=plot(cmd_vels_time,cmd_vels_z,'-g')
    xlabel("Time")
    ylabel("rad/s")
    title("Angular velocity experiment")
    legend([p1,p2,p3],["imu","roboteq cmd vel","arduino cmd vel"])
    f6.show() 


    #print len(cmd_vels_time),len(imu_time)
    #Error analysis
    cmdVels,ImuVels = sync.matchInput(cmd_vels_time,cmd_vels_z,imu_time,imu_z,threshold)

    f7 = figure(7)
    plot(cmdVels,ImuVels,'ob')
    xlabel("Command Velocities (rad/s)")
    ylabel("IMU Velocities (rad/s)")
    title("Experiment cmd imu velocities")
    xlim([-.75,.75])
    ylim([-.75,.75])
    f7.show() 

# bagFile        = "/home/redblade/ROBOTEQDATA/gps_test7/circle.bag"
# front_encoders = "/encoders_front"
# back_encoders  = "/encoders_back"
# front_cmds     = "/roboteq_front/cmd_vel_stamped"
# back_cmds      = "/roboteq_back/cmd_vel_stamped"
# imu            = "/imu/data"
# gps            = "/gps"
# cmd_vels       = "/Arduino_RC"

# go(bagFile,        
#    front_encoders, 
#    back_encoders,  
#    front_cmds,     
#    back_cmds,      
#    imu,            
#    gps,            
#    cmd_vels)
# raw_input()
