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

bagFile = "/home/redblade/ROBOTEQDATA/angular_test3/1.1.bag"
front_encoders = "/encoders_front"
back_encoders = "/encoders_back"
front_cmds = "/roboteq_front/cmd_vel_stamped"
back_cmds = "/roboteq_back/cmd_vel_stamped"
imu = "/imu/data"
gps = "/gps"
cmd_vels = "/cmd_vel"

# plot_velocity.go(bagFile,        
#                  front_encoders, 
#                  back_encoders,  
#                  front_cmds,     
#                  back_cmds,      
#                  imu,            
#                  gps,            
#                  cmd_vels)
os.chdir("/home/redblade/ROBOTEQDATA/angular_test3")
for bagFile in glob.glob("*.bag"):
    print bagFile

    cmdVels,ImuVels=plot_velocity.getAngularVels(bagFile,
                                                 front_encoders, 
                                                 back_encoders,  
                                                 front_cmds,      
                                                 back_cmds,      
                                                 imu,            
                                                 gps,            
                                                 cmd_vels)
    cmdVel = numpy.mean(cmdVels)
    ImuVel = numpy.mean(ImuVels[200:])
    print cmdVel,ImuVel
    plot(cmdVel,ImuVel,"ob")
    
xlabel("Command Velocities (m/s)")
ylabel("Imu Velocities (m/s)")
title("Angular Velocities at 0.5 m/s Linear")
show()
raw_input()
