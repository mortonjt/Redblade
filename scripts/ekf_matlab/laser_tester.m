%laser scanner x,y tester
clear all; close all; clc;

pose = [1,2,pi/6];
range = 2;

%ENU some other position (if pole was straight ahead)
pose_after_straight = [pose(1)+cos(pose(3))*range;
    pose(2)+sin(pose(3))*range];

pose_after_straight_xy = [2,0];

%pole's actual position in ENU
pose_after_to_left = [pose(1)+cos(pose(3)*2)*range;
    pose(2)+sin(pose(3)*2)*range];
pose_after_to_left_xy = [cos(pose(3))*range;
    sin(pose(3))*range];

%wut.jpg
range_check = sqrt((pose_after_to_left(2)-pose(2))^2+(pose_after_to_left(1)-pose(1))^2)
theta_check = atan2((pose_after_to_left(2)-pose(2)),(pose_after_to_left(1)-pose(1)))-pose(3)



scatter(pose(1),pose(2));
hold on;
scatter(pose_after_straight(1),pose_after_straight(2));
scatter(pose_after_to_left(1),pose_after_to_left(2));
%scatter(diff_x,diff_y,'+');
axis([0 4 0 4]);

