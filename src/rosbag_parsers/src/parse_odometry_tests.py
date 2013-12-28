import rosbag
import argparse


"""
Note:
imu_topic must spit out Vector3Stamped
"""


def parse(bag,out,front,back,imu):
    front_out = open("%s.front.cvs"%(out),'w')
    back_out  = open("%s.back.cvs"%(out),'w')
    imu_out   = open("%s.imu.cvs"%(out),'w')

    for topic,msg,t in bag.read_messages(topics=[front,back,imu]):
        if topic==front:
            front_out.write("%d,%d,%d,%d,%d\n"%(msg.header.stamp.secs,
                                                msg.header.stamp.nsecs,
                                                msg.encoders.time_delta,
                                                msg.encoders.left_wheel,
                                                msg.encoders.right_wheel))
        elif topic==back:
            back_out.write("%d,%d,%d,%d,%d\n"%(msg.header.stamp.secs,
                                               msg.header.stamp.nsecs,
                                               msg.encoders.time_delta,
                                               msg.encoders.left_wheel,
                                               msg.encoders.right_wheel))
        elif topic==imu:
            imu_out.write("%d,%d,%d,%d,%d\n"%(msg.header.stamp.secs,
                                              msg.header.stamp.nsecs,
                                              msg.vector.x,
                                              msg.vector.y,
                                              msg.vector.z))
    front_out.close()
    back_out.close() 
    imu_out.close()
    
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
        '--imu_topic',type=str,required=False,default="/imu/integrated_gyros_stamped",
        help='name of imu topic')
    # parser.add_argument(\
    #     '--gps_topic',type=str,required=False,default="/fix",
    #     help='name of gps topic')
    args = parser.parse_args()
    
    
    bag = rosbag.Bag(args.input_rosbag)
    parse(bag,
          args.out,
          args.front_encoder_topic,
          args.back_encoder_topic,
          args.imu_topic)
