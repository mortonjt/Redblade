import rosbag
import argparse


"""
Note:
imu_topic must spit out Vector3Stamped
"""
def parse(bag,out,imu,gps,odom,ekf,ekf2d,lidarpole):
    imu_out           = open("%s.imu.csv"%(out),'w')
    gps_out           = open("%s.gps.csv"%(out),'w')
    odom_out          = open("%s.odom.csv"%(out),'w')
    ekf_out           = open("%s.ekf.csv"%(out),'w')
    ekf_2d_out        = open("%s.ekf2dpose.csv"%(out),'w')
    lidar_pole_out    = open("%s.lidarpole.csv"%(out),'w')

    for topic,msg,t in bag.read_messages(topics=[imu,gps,odom,ekf,ekf2d,lidarpole]):
        if topic==imu:
            imu_out.write("%d,%d,%f,%f,%f\n"%(msg.header.stamp.secs,
                                              msg.header.stamp.nsecs,
                                              msg.vector.x,
                                              msg.vector.y,
                                              msg.vector.z))
        elif topic==gps:
            gps_out.write("%d,%d,%f,%f,%f,"%(msg.header.stamp.secs,
                                             msg.header.stamp.nsecs,
                                             msg.pose.pose.position.x,
                                             msg.pose.pose.position.y,
                                             msg.pose.pose.position.z))
            gps_out.write(",".join( map( str ,msg.pose.covariance))+"\n")
        elif topic==odom:
            odom_out.write("%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,"%(msg.header.stamp.secs,
                                                                msg.header.stamp.nsecs,
                                                                msg.pose.pose.position.x,
                                                                msg.pose.pose.position.y,
                                                                msg.pose.pose.position.z,
                                                                msg.twist.twist.linear.x,
                                                                msg.twist.twist.linear.y,
                                                                msg.twist.twist.linear.z,
                                                                msg.twist.twist.angular.x,
                                                                msg.twist.twist.angular.y,
                                                                msg.twist.twist.angular.z))
            odom_out.write(",".join( map( str ,msg.pose.covariance))+",")
            odom_out.write(",".join( map( str ,msg.twist.covariance))+"\n")
        elif topic==ekf:
            ekf_out.write("%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,"%(msg.header.stamp.secs,
                                                               msg.header.stamp.nsecs,
                                                               msg.pose.pose.position.x,
                                                               msg.pose.pose.position.y,
                                                               msg.pose.pose.position.z,
                                                               msg.twist.twist.linear.x,
                                                               msg.twist.twist.linear.y,
                                                               msg.twist.twist.linear.z,
                                                               msg.twist.twist.angular.x,
                                                               msg.twist.twist.angular.y,
                                                               msg.twist.twist.angular.z))
            ekf_out.write(",".join( map( str ,msg.pose.covariance))+",")
            ekf_out.write(",".join( map( str ,msg.twist.covariance))+"\n")
        elif topic==ekf2d:
            ekf_2d_out.write("%f,%f,%f\n"%(msg.x,
                                           msg.y,
                                           msg.theta))
        elif topic==lidarpole:
            ekf_2d_out.write("%d,%d,%f,%f,%f\n"%(msg.header.stamp.secs,
                                                 msg.header.stamp.nsecs,
                                                 msg.point.x,
                                                 msg.point.y,
                                                 msg.point.z))
    imu_out.close()           
    gps_out.close()
    odom_out.close()  
    ekf_out.close()   
    ekf_2d_out.close()
    lidar_pole_out.close()

    
if __name__=="__main__":
    parser = argparse.ArgumentParser(\
        description="Parses rosbags from odometry tests")
    parser.add_argument(\
        '--input_rosbag',type=str,required=True,default="",
        help='name of input rosbag file')
    parser.add_argument(\
        '--out',type=str,required=False,default="out",
        help='basename of output file(s)')
    parser.add_argument(\
        '--imu_topic',type=str,required=False,default="/imu/integrated_gyros_stamped",
        help='name of imu topic')
    parser.add_argument(\
        '--gps_topic',type=str,required=False,default="/gps",
        help='name of gps topic')
    parser.add_argument(\
        '--odom_topic',type=str,required=False,default="/odom",
        help='name of odom topic')
    parser.add_argument(\
        '--ekf_topic',type=str,required=False,default="/redblade_ekf/odom",
        help='name of ekf topic')
    parser.add_argument(\
        '--ekf_2d_topic',type=str,required=False,default="/redblade_ekf/2d_pose",
        help='name of ekf topic')
    parser.add_argument(\
        '--lidar_pole_topic',type=str,required=False,default="/lidar/pole",
        help='name of pole detected by lidar')
    args = parser.parse_args()
        
    bag = rosbag.Bag(args.input_rosbag)
    parse(bag,
          args.out,
          args.imu_topic,
          args.gps_topic,
          args.odom_topic,
          args.ekf_topic,
          args.ekf_2d_topic,
          args.lidar_pole_topic)
