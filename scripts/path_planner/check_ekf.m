clear all; close all; clc;

%read in cmd_vel
cmd_vel = csvread('ekf/out.back_cmd_vel.csv');
cmd_vel_time = cmd_vel(:,1)+(cmd_vel(:,2)*1e-9);
% subplot(2,1,1);
% plot(cmd_vel_time,cmd_vel(:,8));
% title('Commanded Angular Velocities');
% subplot(2,1,2);
% plot(cmd_vel_time,cmd_vel(:,3));
% title('Commanded Linear Velocities');

%read in gps
gps = csvread('ekf/out.gps.csv');
gps_time = gps(:,1)+(gps(:,2)*1e-9);

subplot(3,1,1);
plot(gps(:,3));
title('East positions vs. time');
subplot(3,1,2);
plot(gps(:,4));
title('North positions vs. time');
subplot(3,1,3);
plot(gps(:,5));
title('Altitude positions vs. time');

figure;
scatter(gps(:,3),gps(:,4));
title('Scatter of GPS EN');
xlabel('East');
ylabel('North');

%read in imu
imu = csvread('ekf/out.imu.csv');
imu_time = imu(:,1)+(imu(:,2)*1e-9);

%read in ekf
ekf = csvread('ekf/out.ekf.csv');
ekf_time = ekf(:,1)+(ekf(:,2)*1e-9);

%read in ekf 2dpose
ekf_2d = csvread('ekf/out.ekf_2d_out.csv');

%read in odom readings
odom = csvread('ekf/out.odom.csv');

%scatter plot of gps vs. kalman filter position
scatter(ekf_2d(:,1),ekf_2d(:,2),'b');
hold on;
scatter(gps(:,3),gps(:,4),'r');

%fake time stamps for 2dpose
n = (length(ekf_2d(:,3)));
ekf_2d_time = imu_time(1):((imu_time(end)-imu_time(1))/(n-1)):imu_time(end);

%time plot of integrated gyros vs. kalman filter orientation
figure; hold on;
plot(imu_time,imu(:,5),'r','LineWidth',2);
plot(ekf_2d_time,ekf_2d(:,3),'b','LineWidth',2);
xlabel('Time(seconds)');
ylabel('Z Orientation (rads)');


n_imu = length(imu(:,5));
n_ekf = length(ekf_2d(:,3));
new_index = round((1:n_ekf)*(n_imu/n_ekf));
%plot bias
bias = ekf_2d(:,3)-imu(new_index,5);
for ii = 1:length(bias)
   if(bias(ii) > pi)
      bias(ii) = bias(ii)-2*pi; 
   elseif(bias(ii) < -pi)
      bias(ii) = bias(ii)+2*pi; 
   end
end
figure;
plot(bias);

%plot of difference between gps position and kalman filter position
%should have constant 21 cm offset
figure;
subplot(2,1,1); hold on;
gps_ = plot(gps_time,gps(:,3),'r','LineWidth',2);
ekf_ = plot(ekf_time,ekf(:,3),'b','LineWidth',2);
legend([gps_ ekf_],'GPS','EKF');
subplot(2,1,2); hold on;
gps_ = plot(gps_time,gps(:,4),'r','LineWidth',2);
ekf_ = plot(ekf_time,ekf(:,4),'b','LineWidth',2);
legend([gps_ ekf_],'GPS','EKF');
