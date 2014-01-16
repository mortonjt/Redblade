clear all; close all; clc;

ekf = csvread('ekf_2/ekf_data_collect_2.txt');
%ekf = ekf(24:end,:);

%label stuff
%time = ekf(:,1);
z1 = ekf(:,2);
z2 = ekf(:,3);
z3 = ekf(:,4);
z4 = ekf(:,5);
z5 = ekf(:,6);

%plot z1 and variance
figure; hold on;
plot(z1,'r','LineWidth', 2);
title('z1 (GPS East)');
xlabel('Sample number');
ylabel('Position (m)');

%plot z2 and variance
figure; hold on;
plot(z2,'r','LineWidth', 2);
title('z2 (GPS North)');
xlabel('Sample number');
ylabel('Position (m)');

%plot z3 and variance
figure; hold on;
plot(z3,'r','LineWidth', 2);
title('z3 (Odom linear vel.)');
xlabel('Sample number');
ylabel('Position (m)');

%plot z4 and variance
figure; hold on;
plot(z4,'r','LineWidth', 2);
title('z4 (IMU heading)');
xlabel('Sample number');
ylabel('Position (m)');

%plot z5 and variance
figure; hold on;
plot(z5,'r','LineWidth', 2);
title('z5 (Odom angular vel.)');
xlabel('Sample number');
ylabel('Position (m)');

x1 = ekf(:,7);
x2 = ekf(:,8);
x3 = ekf(:,9);
x4 = ekf(:,10);
x5 = ekf(:,11);
x6 = ekf(:,12);

offset = 12;
%1,8,15,22,29,36
p1 = ekf(:,offset+1);
p2 = ekf(:,offset+8);
p3 = ekf(:,offset+15);
p4 = ekf(:,offset+22);
p5 = ekf(:,offset+29);
p6 = ekf(:,offset+36);

%plot x1 and variance
figure; hold on;
plot(x1,'b','LineWidth', 2);
plot(x1-(sqrt(p1)),'r','LineWidth',2);
plot(x1+(sqrt(p1)),'r','LineWidth',2);
title('x1 (East)');
xlabel('Sample number');
ylabel('Position (m)');

%plot x2 and variance
figure; hold on;
plot(x2,'b','LineWidth', 2);
title('x2 (North)');
xlabel('Sample number');
ylabel('Position (m)');

%plot x3 and variance
figure; hold on;
plot(x3,'b','LineWidth', 2);
title('x3 (Heading)');
xlabel('Sample number');
ylabel('Heading (rads)');

%plot x4 and variance
figure; hold on;
plot(x4,'b','LineWidth', 2);
title('x4 (Linear Velocity)');
xlabel('Sample number');
ylabel('Velocity (m/s)');

%plot x5 and variance
figure; hold on;
plot(x5,'b','LineWidth', 2);
title('x5 (Angular Velocity)');
xlabel('Sample number');
ylabel('Angular Velocity (rad/s)');

%plot x6 and variance
figure; hold on;
plot(x6,'b','LineWidth', 2);
plot(x6-(sqrt(p6)),'r','LineWidth',2);
plot(x6+(sqrt(p6)),'r','LineWidth',2);
title('x6 (Bias)');
xlabel('Sample number');
ylabel('Bias (rads)');

%plot covariances
figure; hold on;
plot(sqrt(p1),'b','LineWidth',2);
title('x1 Covariance');
xlabel('Sample number');

%plot covariances
figure; hold on;
plot(sqrt(p2),'b','LineWidth',2);
title('x2 Covariance');
xlabel('Sample number');

%plot covariances
figure; hold on;
plot(sqrt(p3),'b','LineWidth',2);
title('x3 Covariance');
xlabel('Sample number');

%plot covariances
figure; hold on;
plot(sqrt(p4),'b','LineWidth',2);
title('x4 Covariance');
xlabel('Sample number');

%plot covariances
figure; hold on;
plot(sqrt(p5),'b','LineWidth',2);
title('x5 Covariance');
xlabel('Sample number');

%plot covariances
figure; hold on;
plot(sqrt(p6),'b','LineWidth',2);
title('x6 Covariance');
xlabel('Sample number');


figure;hold on;
plot(x4,'r','LineWidth',2);
plot(z3,'b','LineWidth',2);
%find diff betwen gps and x1, x2
% diff_e = x1-z1;
% diff_n = x2-z2;
% figure; hold on;
% plot(diff_e,'LineWidth',2);
% 
% figure; hold on;
% plot(diff_n,'LineWidth',2);
