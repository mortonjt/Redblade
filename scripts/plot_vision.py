from pylab import *
import rosbag
import numpy
import bisect
import math

import movingAverage

#gpsBag = "/home/jamie/Documents/data/test4/pole.bag"
#poleBag = "/home/jamie/Documents/data/lidar.bag"
poleBag = "/home/jamie/DATA/LIDAR/lidar_processed.bag"
front_encoders = "/encoders_front"
back_encoders = "/encoders_back"
front_cmds = "/roboteq_front/cmd_vel_stamped"
back_cmds = "/roboteq_back/cmd_vel_stamped"
imu = "/imu/data"
gps = "/gps"
cmd_vels = "/cmd_vel"
lidar_pole = "/lidar/pole"
stereo_pole = "/stereo_camera/pole"
pose2D = "/redblade_ekf/2d_pose"

#bag = rosbag.Bag(gpsBag)
#threshold = 0.1
#gps_msgs          = [msg for topic,msg,t in bag.read_messages(topics=[gps])]

#gps_x = [x.pose.pose.position.x for x in gps_msgs]
#gps_y = [y.pose.pose.position.y for y in gps_msgs]
#time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9 for t in gps_msgs]


#print min(time),max(time),len(time)
#print "Average GPS position",numpy.mean(gps_x[-50:]),numpy.mean(gps_y[-50:])

correct_x = -11.4
correct_y = -4.7

bag = rosbag.Bag(poleBag)
threshold = 0.1
gps_msgs           = [msg for topic,msg,t in bag.read_messages(topics=[gps])]
lidar_msgs    = [msg for topic,msg,t in bag.read_messages(topics=[lidar_pole])]
stereo_msgs   = [msg for topic,msg,t in bag.read_messages(topics=[stereo_pole])]
pose2D_msgs   = [msg for topic,msg,t in bag.read_messages(topics=[pose2D])]

L = min([len(gps_msgs),len(lidar_msgs)])
gps_x = [x.pose.pose.position.x for x in gps_msgs][:L]
gps_y = [y.pose.pose.position.y for y in gps_msgs][:L]
lidar_pole_x = [x.point.x for x in lidar_msgs][:L]
lidar_pole_y = [y.point.y for y in lidar_msgs][:L]
stereo_pole_x = [x.point.x for x in stereo_msgs][:L]
stereo_pole_y = [y.point.y for y in stereo_msgs][:L]
time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9 for t in gps_msgs][:L]

pose2D_msgs   = [msg for topic,msg,t in bag.read_messages(topics=[pose2D])][:L]

print "lidar X std dev",numpy.std(lidar_pole_x[1:])
print "lidar Y std dev",numpy.std(lidar_pole_y[1:])

d = [math.sqrt((lidar_pole_y[i]-correct_y)**2 +
               (lidar_pole_x[i]-correct_x)**2) for i in xrange(len(lidar_pole_y))]

f1 = figure(1)
p1,=plot(lidar_pole_x,lidar_pole_y,'ob')
p2,=plot(gps_x,gps_y,'og')

p3,=plot(correct_x,correct_y,'or')
ylabel("x")
xlabel("y")
title("Experiment positions")
legend([p1,p2,p3],["Lidar estimates"," GPS estimates","Correct position"])
f1.show() 

print len(time),len(lidar_pole_x)
f2 = figure(2)
p1,=plot(time,gps_x,'og')
p2,=plot(time,lidar_pole_x,'ob')
p3,=plot(time,[correct_x]*len(time),'or')
ylabel("Distance (x)")
xlabel("time")
title("Position vs time of Lidar measurements on x axis")
legend([p1,p2,p3],["GPS Robot position"," Pole position (LIDAR)","Correct position"])
f2.show() 

f3 = figure(3)
p1,=plot(time,gps_y,'og')
p2,=plot(time,lidar_pole_y,'ob')
p3,=plot(time,[correct_y]*len(time),'or')
ylabel("Distance (y)")
xlabel("time")
title("Position vs time of Lidar measurements on y axis")
legend([p1,p2,p3],["GPS Robot position"," Pole position (LIDAR)","Correct position"])
f3.show() 


# print "stereo X std dev",numpy.std(stereo_pole_x[1:])
# print "stereo Y std dev",numpy.std(stereo_pole_y[1:])

# f4 = figure(4)
# p1,=plot(stereo_pole_y,stereo_pole_x,'ob')
# p2,=plot(correct_y,correct_x,'or')
# ylabel("x")
# xlabel("y")
# title("Experiment positions")
# legend([p1,p2],["Stereo camera estimates"," GPS estimates"])
# f4.show() 

# print len(stereo_pole_x),len(time)
# f5 = figure(5)
# p1,=plot(time,gps_x,'og')
# p2,=plot(time,stereo_pole_x,'ob')
# p3,=plot(time,[correct_x]*len(time),'or')
# ylabel("Distance (x)")
# xlabel("time")
# title("Position vs time of Stereo measurements on x axis")
# legend([p1,p2,p3],["GPS Robot position"," Pole position (STEREO)","Correct position"])
# f5.show() 

# f6 = figure(6)
# p1,=plot(time,gps_y,'og')
# p2,=plot(time,stereo_pole_y,'ob')
# p3,=plot(time,[correct_y]*len(time),'or')
# ylabel("Distance (y)")
# xlabel("time")
# title("Position vs time of Stereo measurements on y axis")
# legend([p1,p2,p3],["GPS Robot position"," Pole position (STEREO)","Correct position"])
# f6.show() 


# print len(time),len(headings)
# f7 = figure(7)
# plot(time,headings,'og')
# ylabel("Rad (y)")
# xlabel("time")
# title("heading vs time of Stereo measurements on y axis")
# #legend([p1,p2,p3],["GPS Robot position"," Pole position (STEREO)","Correct position"])
# f7.show() 

raw_input()
