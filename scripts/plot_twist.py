from pylab import *
import rosbag

clicks_per_m = 15768.6


bagFile = "/home/jamie/Downloads/twist_converter_test5/2013-12-29-14-04-03.bag"
front_encoders = "/roboteq_front/encoders"
back_encoders = "/roboteq_back/encoders"
front_cmds = "/roboteq_front/cmd_vel_stamped"
back_cmds = "/roboteq_back/cmd_vel_stamped"
imu = "/imu/integrated_gyros_stamped"
bag = rosbag.Bag(bagFile)

front_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[front_encoders])]
back_encoder_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_encoders])]
front_cmd_msgs = [msg for topic,msg,t in bag.read_messages(topics=[front_cmds])]
back_cmd_msgs = [msg for topic,msg,t in bag.read_messages(topics=[back_cmds])]
imu_msgs = [msg for topic,msg,t in bag.read_messages(topics=[imu])]

imu_time           = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in imu_msgs]
front_encoder_time = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in front_encoder_msgs]
back_encoder_time  = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in back_encoder_msgs]
front_cmd_time     = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in front_cmd_msgs]
back_cmd_time      = [t.header.stamp.secs+t.header.stamp.nsecs*10**-9 for t in back_cmd_msgs]

front_cmd_z        = [t.twist.angular.z for t in front_cmd_msgs]
back_cmd_z         = [t.twist.angular.z for t in back_cmd_msgs]
imu_z              = [t.vector.z for t in imu_msgs]

plot(front_cmd_time,front_cmd_z,'-b')
plot(back_cmd_time,back_cmd_z,'-g')
plot(imu_time,imu_z,'-r')
show() 
