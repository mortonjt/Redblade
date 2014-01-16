clear all; close all; clc;

%read gps_file
raw_gps = csvread('line/line.gps.csv');
east = raw_gps(:,3);
north = raw_gps(:,4);

%plot gps stuff
subplot(2,1,1);
plot(east);
title('Easting and Northing vs. Time');
xlabel('Time (samples)');
ylabel('Easting (meters)');
subplot(2,1,2);
plot(north);
xlabel('Time (samples)');
ylabel('Northing (meters)');

%calclate velocities
east_vel = [0; (east(2:end)-east(1:(end-1)))];
figure; subplot(2,1,1);
plot(east_vel);
title('East and North Velocities');
xlabel('Time (samples)');
ylabel('Velocity(m/s)');
north_vel = [0; (north(2:end)-north(1:(end-1)))];
subplot(2,1,2);
plot(north_vel);
xlabel('Time (samples)');
ylabel('Velocity(m/s)');

%plot total velocity
total_vel = sqrt(east_vel.^2+north_vel.^2);
figure;
plot(total_vel);
title('GPS velocity vs. Time');
xlabel('Time (samples)');
ylabel('Velocity (m/s)');

%plot scatter of gps position
figure;
scatter(east, north);
xlabel('Easting');
ylabel('Northing');
title('GPS positions');

%nanoseconds
% nano = raw_gps(:,2);
% for ii = 2:length(nano)
%    nano(ii) = nano(ii-1)+nano(ii);
% end
% figure;
% plot(nano);