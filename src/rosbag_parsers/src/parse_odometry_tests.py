import rosbag
import argparse


"""
Note:
imu_topic must spit out Vector3Stamped
"""
def parse(bag,out,front_encoder,back_encoder,front_cmd_vel,back_cmd_vel,imu,gps):
    front_encoder_out = open("%s.front_encoder.cvs"%(out),'w')
    back_encoder_out  = open("%s.back_encoder.cvs"%(out),'w')
    front_cmd_out     = open("%s.front_cmd_vel.cvs"%(out),'w')
    back_cmd_out      = open("%s.back_cmd_vel.cvs"%(out),'w')
    imu_out           = open("%s.imu.cvs"%(out),'w')
    gps_out           = open("%s.gps.cvs"%(out),'w')

    for topic,msg,t in bag.read_messages(topics=[front_encoder,back_encoder,front_cmd_vel,back_cmd_vel,imu,gps]):
        if topic==front_encoder:
            front_encoder_out.write("%d,%d,%d,%d,%d\n"%(msg.header.stamp.secs,
                                                        msg.header.stamp.nsecs,
                                                        msg.encoders.time_delta,
                                                        msg.encoders.left_wheel,
                                                        msg.encoders.right_wheel))
        elif topic==back_encoder:
            back_encoder_out.write("%d,%d,%d,%d,%d\n"%(msg.header.stamp.secs,
                                                       msg.header.stamp.nsecs,
                                                       msg.encoders.time_delta,
                                                       msg.encoders.left_wheel,
                                                       msg.encoders.right_wheel))
        if topic==front_cmd_vel:
            front_cmd_out.write("%f,%f,%f,%f,%f,%f\n"%(msg.linear.x,
                                                       msg.linear.y,
                                                       msg.linear.z,
                                                       msg.angular.x,
                                                       msg.angular.y,
                                                       msg.angular.z))
        elif topic==back_cmd_vel:
            back_cmd_out.write("%f,%f,%f,%f,%f,%f\n"%(msg.linear.x,
                                                      msg.linear.y,
                                                      msg.linear.z,
                                                      msg.angular.x,
                                                      msg.angular.y,
                                                      msg.angular.z))
        elif topic==imu:
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
            gps_out.write(",".join( map( str ,msg.twist.covariance))+"\n")

    front_encoder_out.close()
    back_encoder_out.close()  
    front_cmd_out.close()                 
    back_cmd_out.close()      
    imu_out.close()           
    gps_out.close()
    
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
        '--front_encoder_topic',type=str,required=False,default="/roboteq_front/encoders",
        help='name of front encoder topic')
    parser.add_argument(\
        '--back_encoder_topic',type=str,required=False,default="/roboteq_back/encoders",
        help='name of back encoder topic')
    parser.add_argument(\
        '--front_cmd_vel_topic',type=str,required=False,default="/roboteq_front/cmd_vel",
        help='name of front cmd_vel topic')
    parser.add_argument(\
        '--back_cmd_vel_topic',type=str,required=False,default="/roboteq_back/cmd_vel",
        help='name of back cmd_vel topic')
    parser.add_argument(\
        '--imu_topic',type=str,required=False,default="/imu/integrated_gyros_stamped",
        help='name of imu topic')
    parser.add_argument(\
        '--gps_topic',type=str,required=False,default="/gps",
        help='name of gps topic')
    args = parser.parse_args()
        
    bag = rosbag.Bag(args.input_rosbag)
    parse(bag,
          args.out,
          args.front_encoder_topic,
          args.back_encoder_topic,
          args.front_cmd_vel_topic,
          args.back_cmd_vel_topic,
          args.imu_topic,
          args.gps_topic)
