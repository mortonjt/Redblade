import math
clicks_per_m = 15768.6;

def calculateAngularImuVelocity(paired_msgs):
    prev_msg,cur_msg = paired_msgs
    prev_z,prev_time = prev_msg
    cur_z,cur_time = cur_msg
    dTime = float(cur_time-prev_time)%math.pi
    return (cur_z-prev_z)/dTime

def getAngularImuVelocities(imu_z,time):
    msgs  = zip(imu_z,time)
    pairs = zip(msgs[:-1],msgs[1:])
    return map(calculateAngularImuVelocity,pairs)

def calculateLinearGPSVelocity(paired_msgs):
    prev_msg,cur_msg = paired_msgs
    prev_x,prev_y,prev_time = prev_msg
    cur_x,cur_y,cur_time = cur_msg
    dTime = float(cur_time-prev_time)
    distance = math.sqrt( (cur_x-prev_x)*(cur_x-prev_x) + \
                          (cur_y-prev_y)*(cur_y-prev_y) )
    return distance/dTime

def getLinearGPSVelocities(gps_x,gps_y,time):
    msgs  = zip(gps_x,gps_y,time)
    pairs = zip(msgs[:-1],msgs[1:])
    return map(calculateLinearGPSVelocity,pairs)


def calculateLinearEncoderVelocity(paired_msgs):
    prev_msg,cur_msg = paired_msgs
    prev_left_encoders,prev_right_encoders,prev_time = prev_msg
    cur_left_encoders,cur_right_encoders,cur_time = cur_msg
    dTime = cur_time-prev_time
    prev_left_pos  = prev_left_encoders/clicks_per_m
    prev_right_pos = prev_right_encoders/clicks_per_m
    cur_left_pos  = cur_left_encoders/clicks_per_m
    cur_right_pos = cur_right_encoders/clicks_per_m
    left_vel = (cur_left_pos-prev_left_pos)/dTime
    right_vel = (cur_right_pos-prev_right_pos)/dTime
    return (left_vel+right_vel)/2.0

def getLinearEncoderVelocities(front_encoders,back_encoders):
    front_left = [x.encoders.left_wheel  for x in front_encoders]
    front_right= [x.encoders.right_wheel for x in front_encoders]
    back_left  = [x.encoders.left_wheel  for x in back_encoders]
    back_right = [x.encoders.right_wheel for x in back_encoders]
    front_time = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
                  for t in front_encoders]
    back_time  = [t.header.stamp.secs+t.header.stamp.nsecs/10.0**9
                  for t in back_encoders]
    front_msgs = zip(front_left,front_right,front_time)
    front_pairs= zip(front_msgs[:-1],front_msgs[1:])
    back_msgs  = zip(back_left,back_right,back_time)
    back_pairs = zip(back_msgs[:-1],back_msgs[1:])
    frontVels  = map(calculateLinearEncoderVelocity,front_pairs)
    backVels   = map(calculateLinearEncoderVelocity,back_pairs)
    return frontVels,backVels
    
