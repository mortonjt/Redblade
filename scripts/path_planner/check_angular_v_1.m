clear all; close all; clc;

count = 1;
%read in files for linear velocity of 0.5
for ii = 0.05:0.05:1.05
    if(ii == 1)
        filename_cmd_vel = ['csvFiles\1.0_1.0.back_cmd_vel.csv'];
        filename_imu = ['csvFiles\1.0_1.0.imu.csv'];
    else
        filename_cmd_vel = ['csvFiles\1.0_' num2str(ii) '.back_cmd_vel.csv'];
        filename_imu = ['csvFiles\1.0_' num2str(ii) '.imu.csv'];
    end
    
    cmd_vel(count).data = csvread(filename_cmd_vel);
    imu(count).data = csvread(filename_imu);
    count = count + 1;
end

vels = 0.05:0.05:1.05;
for ii = 1:length(vels)
    time = cmd_vel(ii).data(:,1);
    nsec = cmd_vel(ii).data(:,2);
    time_vel = time + (nsec*1e-9);
    linear = cmd_vel(ii).data(:,3);
    angular_cmd = cmd_vel(ii).data(:,8);
    
    time = imu(ii).data(:,1);
    nsec = imu(ii).data(:,2);
    time_imu = time + (nsec*1e-9);
    
    angular_imu = imu(ii).data(:,5);
    angular_imu = [0; angular_imu(2:end)-angular_imu(1:end-1)]*100;
    
    %check for rollover
    for jj = 1:length(angular_imu)
       if(angular_imu(jj) > 2)
          angular_imu(jj) = 0; 
       elseif(angular_imu(jj) < -2)
           angular_imu(jj) = 0;
       end
    end
    
    %delete first n data points
    time_imu = time_imu(300:end);
    angular_imu = angular_imu(300:end);
    
    avg_imu(ii) = mean(angular_imu);
    
    figure;
    plot(time_vel,angular_cmd,time_imu,angular_imu);
    title(['Imu data for cmd_vel ' num2str(vels(ii)) ' rad/s']);
    xlabel('Time(s)');
    ylabel('Rad/s');
    
end

%take off the last four data points because i don't like them
vels = vels(1:end-5);
avg_imu = avg_imu(1:end-5);

figure;
scatter(avg_imu,vels);
title('Cmd_vel vs. Imu Vel.');
xlabel('cmd_vel');
ylabel('Velocity (rad/s)');
