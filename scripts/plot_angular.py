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
folder = "/home/jamie/Downloads/angular_test3"
os.chdir(folder)

angularData = []
f1 = figure(1)
for bagFile in glob.glob("0.5_*.bag"):
    #print bagFile

    cmdVels,ImuVels=plot_velocity.getAngularVels(bagFile,
                                                 front_encoders, 
                                                 back_encoders,  
                                                 front_cmds,      
                                                 back_cmds,      
                                                 imu,            
                                                 gps,            
                                                 cmd_vels)

    cmdVel = numpy.mean(cmdVels[300:])
    ImuVel = numpy.mean(ImuVels[300:])
    #data = zip(cmdVels[200:-100],ImuVels[200:-100])
    #data = zip(cmdVels[50:],ImuVels[50:])
    data = [(cmdVel,ImuVel)]
    angularData+=data
    #plot(cmdVel,ImuVel,"ob")

cmdVel,ImuVel = zip(*angularData)
cmdVel = numpy.array(cmdVel[:-5])
ImuVel = numpy.array(ImuVel[:-5])
A = np.vstack([ImuVel, np.ones(len(ImuVel))]).T
m, c = np.linalg.lstsq(A, cmdVel)[0]
print "y=%f x + %f"%(m,c)
plot(ImuVel,cmdVel, 'o', label='Original data', markersize=10)
plot(ImuVel, m*ImuVel + c, 'r', label='Fitted line')

#for vel in angularData:
#    plot(vel[0],vel[1],"ob")

handle = open("%s/%s"%(folder,"lin0.5.csv"),'w')
handle.write("\n".join(["%f\t%f"%(x[0],x[1]) for x in angularData]))
ylabel("Command Velocities (m/s)")
xlabel("Imu Velocities (m/s)")
title("Angular Velocities at 0.5 m/s Linear")
f1.show()


f2 = figure(2)
angularData = []
for bagFile in glob.glob("1.0_*.bag"):
    #print bagFile

    cmdVels,ImuVels=plot_velocity.getAngularVels(bagFile,
                                                 front_encoders, 
                                                 back_encoders,  
                                                 front_cmds,      
                                                 back_cmds,      
                                                 imu,            
                                                 gps,            
                                                 cmd_vels)

    cmdVel = numpy.mean(cmdVels[300:])
    ImuVel = numpy.mean(ImuVels[300:])
    data = [(cmdVel,ImuVel)]
    #data = zip(cmdVels[200:-100],ImuVels[200:-100])
    angularData+=data

# for vel in angularData:
#     plot(vel[0],vel[1],"ob")

cmdVel,ImuVel = zip(*angularData)
cmdVel = numpy.array(cmdVel[:-5])
ImuVel = numpy.array(ImuVel[:-5])
A = np.vstack([ImuVel, np.ones(len(ImuVel))]).T
m, c = np.linalg.lstsq(A, cmdVel)[0]
print "y=%f x + %f"%(m,c)
plot(ImuVel,cmdVel, 'o', label='Original data', markersize=10)
plot(ImuVel, m*ImuVel + c, 'r', label='Fitted line')

    
handle = open("%s/%s"%(folder,"lin1.0.csv"),'w')
handle.write("\n".join(["%f\t%f"%(x[0],x[1]) for x in angularData]))
ylabel("Command Velocities (m/s)")
xlabel("Imu Velocities (m/s)")
title("Angular Velocities at 1.0 m/s Linear")
f2.show()


raw_input()
