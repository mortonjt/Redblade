clear all; close all; clc;
extension = 'test';

%todo:
%fix the odometry angular velocities

%real_data = csvread('real_robot_data/ekf_data_collect_2.txt');

%number of seconds
length = 50;

%number of realizations
num_realizations = 1;

%TODO: howfix?
%propogate dt (seconds)
prop_dt = .2;

num_states = 8;
num_measures = 7;


%dynamics params
sigma_w1 = sqrt(.0025);%.0025
sigma_w2 = sqrt(.0025);%.0025
sigma_w3 = sqrt(.0001);%.0001
sigma_w4 = sqrt(.1);%.1
sigma_w5 = sqrt(.0625);%.0625
sigma_w6 = sqrt(.0001156);%.000156
sigma_w7 = sqrt(.0001);%TODO:not totally sure about dese numbas
sigma_w8 = sqrt(.0001);
sigma_w9 = sqrt(.0001);
sigma_w10 = sqrt(.0001);

beta_linear = .1;%.883
beta_angular = .1;%.883

%measurement params
sigma_z1 = sqrt(.0025);%.0025
sigma_z2 = sqrt(.0025);%.0025
sigma_z3 = sqrt(.0025);%.0025
sigma_z4 = sqrt(.0000076);%.0001
sigma_z5 = sqrt(.0025);%.0025
sigma_z6 = sqrt(.0025);%TODO: these numbers will need to change based on sensor noise
sigma_z7 = sqrt(.0025);
sigma_z8 = sqrt(.0025);
sigma_z9 = sqrt(.0025);

%save stuff to plot later
previous_x_k = zeros(num_realizations, num_states, (length/prop_dt));
previous_z_k = zeros(num_realizations, num_measures, (length/prop_dt));
previous_x_k_hat = zeros(num_realizations, num_states, (length/prop_dt));
previous_P_k = zeros(num_realizations, num_states, (length/prop_dt));

%heading offset count (the number of times we need to multiply by 360)
heading_offset = 0;

for realization = 1:num_realizations
    %generate initial gyro bias
    gyro_bias = 1.57*randn(1);
    
    %set inital conditions (truth values)
    x_k = [	0;...
        0;...
        -pi;...%may want to change this to test gyro stuff
        1;...
        0;...
        gyro_bias;...
        -10;...%TODOthese numbers will need to come from a "truth" reference
        -4;...
        -5;...
        -6];
    x_k = x_k(1:num_states);
    
    %set initial estimates
    x_k_hat = [	0.01;...%this won't be zero in actual implementation
        -.10;...%this won't be zero in actual implementation
        -3.1;...
        1.12;...
        0;...
        (gyro_bias+0.1);...
        -5;...%TODO:should these numbers be based on field orientation?
        -5;...
        0.5;...
        -0.5];
    x_k_hat = x_k_hat(1:num_states);
    
    %set initial estimate uncertainty
    P_k = zeros(10,10);
    P_k(1,1) = 0.25;
    P_k(2,2) = 0.25;
    P_k(3,3) = .01;
    P_k(4,4) = .01;%TODO:watdo
    P_k(5,5) = .01;
    P_k(6,6) = .01;
    P_k(7,7) = 100;%TODO:should these numbers be based on the field orientation?
    P_k(8,8) = 4;
    P_k(9,9) = 400;
    P_k(10,10) = 400;
    P_k = P_k(1:num_states,1:num_states);
    
    %F
    if(num_states == 6)
        f_x=@(x,dt)[x(1)+(cos(x(3))*x(4)*dt);...
            x(2)+(sin(x(3))*x(4)*dt);...
            x(3)+(x(5)*dt);...
            x(4)*exp(-beta_linear*dt);...
            x(5)*exp(-beta_angular*dt);...
            x(6)];
        f_prime=@(x,dt)[1 0 -sin(x(3))*x(4)*dt cos(x(3))*dt 0 0;...
            0 1 cos(x(3))*x(4)*dt sin(x(3))*dt 0 0;...
            0 0 1 0 dt 0;...
            0 0 0 exp(-beta_linear*dt) 0 0;...
            0 0 0 0 exp(-beta_angular*dt) 0;...
            0 0 0 0 0 1];
        
        
        
        %H
        h_x=@(x)[x(1) + cos(x(3))*(0.21);...
            x(2) + sin(x(3))*(0.21);...
            x(4);...
            x(3)+x(6);...
            x(5)];
        H_prime=@(x) [1 0 (-sin(x(3))*(0.21)) 0 0 0;...
            0 1 (cos(x(3))*(0.21)) 0 0 0;...
            0 0 0 1 0 0;...
            0 0 1 0 0 1;...
            0 0 0 0 1 0];
        
    elseif(num_states == 8)
        f_x=@(x,dt)[x(1)+(cos(x(3))*x(4)*dt);...
            x(2)+(sin(x(3))*x(4)*dt);...
            x(3)+(x(5)*dt);...
            x(4)*exp(-beta_linear*dt);...
            x(5)*exp(-beta_angular*dt);...
            x(6);...
            x(7);...
            x(8)];
        f_prime=@(x,dt)[1 0 -sin(x(3))*x(4)*dt cos(x(3))*dt 0 0 0 0;...
            0 1 cos(x(3))*x(4)*dt sin(x(3))*dt 0 0 0 0;...
            0 0 1 0 dt 0 0 0;...
            0 0 0 exp(-beta_linear*dt) 0 0 0 0;...
            0 0 0 0 exp(-beta_angular*dt) 0 0 0;...
            0 0 0 0 0 1 0 0;...
            0 0 0 0 0 0 1 0;...
            0 0 0 0 0 0 0 1];
        
        
        
        %H
        h_x=@(x)[	x(1) + cos(x(3))*(0.21);...
            x(2) + sin(x(3))*(0.21);...
            x(4);...
            x(3)+x(6);...
            x(5);...
            sqrt((x(8)-x(2))^2+(x(7)-x(1))^2);...
            atan2((x(8)-x(2)),(x(7)-x(1)))-x(3)];
        H_prime=@(x) [1 0 (-sin(x(3))*(0.21)) 0 0 0 0 0;...
            0 1 (cos(x(3))*(0.21)) 0 0 0 0 0;...
            0 0 0 1 0 0 0 0;...
            0 0 1 0 0 1 0 0;...
            0 0 0 0 1 0 0 0;...
            ((x(1)-x(7))/sqrt((x(8)-x(2)).^2+(x(7)-x(1)).^2)) ...
            ((x(2)-x(8))/sqrt((x(8)-x(2)).^2+(x(7)-x(1)).^2)) ...
            0 0 0 0 ...
            ((x(7)-x(1))/sqrt((x(8)-x(2)).^2+(x(7)-x(1)).^2)) ...
            ((x(8)-x(2))/sqrt((x(8)-x(2)).^2+(x(7)-x(1)).^2));...
            ((x(8)-x(2))/((x(8)-x(2)).^2+(x(7)-x(1)).^2)) ...
            ((x(1)-x(7))/((x(8)-x(2)).^2+(x(7)-x(1)).^2)) ...
            -1 0 0 0 ...
            ((x(2)-x(8))/((x(8)-x(2)).^2+(x(7)-x(1)).^2)) ...
            ((x(7)-x(1))/((x(8)-x(2)).^2+(x(7)-x(1)).^2))];
    elseif(num_states == 10)
        f_x=@(x,dt)[x(1)+(cos(x(3))*x(4)*dt);...
            x(2)+(sin(x(3))*x(4)*dt);...
            x(3)+(x(5)*dt);...
            x(4)*exp(-beta_linear*dt);...
            x(5)*exp(-beta_angular*dt);...
            x(6);...
            x(7);...
            x(8);...
            x(9);...
            x(10)];
        f_prime=@(x,dt)[1 0 -sin(x(3))*x(4)*dt cos(x(3))*dt 0 0 0 0 0 0;...
            0 1 cos(x(3))*x(4)*dt sin(x(3))*dt 0 0 0 0 0 0;...
            0 0 1 0 dt 0 0 0 0 0;...
            0 0 0 exp(-beta_linear*dt) 0 0 0 0 0 0;...
            0 0 0 0 exp(-beta_angular*dt) 0 0 0 0 0;...
            0 0 0 0 0 1 0 0 0 0;...
            0 0 0 0 0 0 1 0 0 0;...
            0 0 0 0 0 0 0 1 0 0;...
            0 0 0 0 0 0 0 0 1 0;...
            0 0 0 0 0 0 0 0 0 1];
        
        
        
        %H
        h_x=@(x)[	x(1) + cos(x(3))*(0.21);...
            x(2) + sin(x(3))*(0.21);...
            x(4);...
            x(3)+x(6);...
            x(5);...
            sqrt((x(8)-x(2))^2+(x(7)-x(1))^2);...
            atan2((x(8)-x(2)),(x(7)-x(1)))-x(3);...
            sqrt((x(10)-x(2))^2+(x(9)-x(1))^2);...
            atan2((x(10)-x(2)),(x(9)-x(1)))-x(3)];
        H_prime=@(x) [1 0 (-sin(x(3))*(0.21)) 0 0 0 0 0 0 0;...
            0 1 (cos(x(3))*(0.21)) 0 0 0 0 0 0 0;...
            0 0 0 1 0 0 0 0 0 0;...
            0 0 1 0 0 1 0 0 0 0;...
            0 0 0 0 1 0 0 0 0 0;...
            ((x(1)-x(7))/sqrt((x(8)-x(2))^2+(x(7)-x(1))^2)) ...
            ((x(2)-x(8))/sqrt((x(8)-x(2))^2+(x(7)-x(1))^2)) ...
            0 0 0 0 ...
            ((x(7)-x(1))/sqrt((x(8)-x(2))^2+(x(7)-x(1))^2)) ...
            ((x(8)-x(2))/sqrt((x(8)-x(2))^2+(x(7)-x(1))^2)) ...
            0 0;...
            ((x(8)-x(2))/((x(8)-x(2))^2+(x(7)-x(1))^2)) ...
            ((x(1)-x(7))/((x(8)-x(2))^2+(x(7)-x(1))^2)) ...
            -1 0 0 0 ...
            ((x(2)-x(8))/((x(8)-x(2))^2+(x(7)-x(1))^2)) ...
            ((x(7)-x(1))/((x(8)-x(2))^2+(x(7)-x(1))^2)) ...
            0 0;...
            ((x(1)-x(9))/sqrt((x(10)-x(2))^2+(x(9)-x(1))^2)) ...
            ((x(2)-x(10))/sqrt((x(10)-x(2))^2+(x(9)-x(1))^2)) ...
            0 0 0 0 0 0 ...
            ((x(9)-x(1))/sqrt((x(10)-x(2))^2+(x(9)-x(1))^2)) ...
            ((x(10)-x(2))/sqrt((x(10)-x(2))^2+(x(9)-x(1))^2));
            ((x(10)-x(2))/((x(10)-x(2))^2+(x(9)-x(1))^2)) ...
            ((x(1)-x(9))/((x(10)-x(2))^2+(x(9)-x(1))^2)) ...
            -1 0 0 0 0 0 ...
            ((x(2)-x(10))/((x(10)-x(2))^2+(x(9)-x(1))^2)) ...
            ((x(9)-x(1))/((x(10)-x(2))^2+(x(9)-x(1))^2))
            ];
    end
    
    %v matrix
    v_k = [	sigma_z1;
        sigma_z2;
        sigma_z3;
        sigma_z4;
        sigma_z5;
        sigma_z6;
        sigma_z7;
        sigma_z8;
        sigma_z9];
    v_k = v_k(1:num_measures);
    
    %R matrix
    R_k = [ sigma_z1^2 0 0 0 0 0 0 0 0;...
        0 sigma_z2^2 0 0 0 0 0 0 0;...
        0 0 sigma_z3^2 0 0 0 0 0 0;...
        0 0 0 sigma_z4^2 0 0 0 0 0;...
        0 0 0 0 sigma_z5^2 0 0 0 0;...
        0 0 0 0 0 sigma_z6^2 0 0 0;...
        0 0 0 0 0 0 sigma_z7^2 0 0;...
        0 0 0 0 0 0 0 sigma_z8^2 0;...
        0 0 0 0 0 0 0 0 sigma_z9^2];
    R_k = R_k(1:num_measures,1:num_measures);
    
    %Q matrix
    Q_k = [ sigma_w1^2 0 0 0 0 0 0 0 0 0;...
        0 sigma_w2^2 0 0 0 0 0 0 0 0;...
        0 0 sigma_w3^2 0 0 0 0 0 0 0;...
        0 0 0 sigma_w4^2*(1-exp(-2*beta_linear*prop_dt)) 0 0 0 0 0 0;...
        0 0 0 0 sigma_w5^2*(1-exp(-2*beta_angular*prop_dt)) 0 0 0 0 0;...
        0 0 0 0 0 sigma_w6^2 0 0 0 0;...
        0 0 0 0 0 0 sigma_w7^2 0 0 0;...
        0 0 0 0 0 0 0 sigma_w8^2 0 0;...
        0 0 0 0 0 0 0 0 sigma_w9^2 0;...
        0 0 0 0 0 0 0 0 0 sigma_w10^2];
    Q_k = Q_k(1:num_states,1:num_states);
    
    for ii = 1:(length/prop_dt)
        %debugging stuff
        error_check = abs(x_k - x_k_hat);
   

        %save realizations to plot later
        previous_x_k(realization, :, ii) = x_k;
        
%         %plug in current estimate
%         H_temp = H_prime(x_k_hat);
%         
%         %compute Kalman Gain
%         K_k = P_k * H_temp' * inv(H_temp * P_k * H_temp' + R_k);
        
        %simulate sensor measurements
        z_k = h_x(x_k) + v_k .*randn(num_measures,1);
        
        %save measurements
        previous_z_k(realization, :, ii) = z_k;
        %save x_k_hat and error covariance
        previous_x_k_hat(realization, :, ii) = x_k_hat;
        previous_P_k(realization, :, ii) = diag(P_k);
        
        if(ii < 50)%if the heading isn't initialized
            H_temp = H_prime(x_k_hat);
            H_temp = H_temp(1:5,1:6);
            
            %compute Kalman Gain
            K_k = P_k(1:6,1:6) * H_temp' / (H_temp * P_k(1:6,1:6) * H_temp' + R_k(1:5,1:5));
            
            %only grab sensor measuremeants 1-5
            z_k = z_k(1:5);
            
            %update estimate with measure z_k
            prediction = h_x(x_k_hat);
            prediction = prediction(1:5);
            x_k_hat(1:6) = x_k_hat(1:6) + K_k * (z_k - prediction);
            
            %compute error covariance for updated estimate
            P_k(1:6,1:6) = (eye(6) - K_k * H_temp) * P_k(1:6,1:6) * ...
            (eye(6) - K_k * H_temp)' + ...
            K_k*R_k(1:5,1:5)*K_k';
            
            
        else
            %ii
            %plug in current estimate
            H_temp = H_prime(x_k_hat);
            
            %compute Kalman Gain
            K_k = P_k * H_temp' / (H_temp * P_k * H_temp' + R_k);
            
            %update estimate with measure z_k
            prediction = h_x(x_k_hat);
            %wrap the residual (don't ask why)
            residual = z_k-prediction;
            if(residual(7) < -pi)
                residual(7) = residual(7) + 2*pi;
            elseif(residual(7) > pi)
                residual(7) = residual(7) - 2*pi;
            end
            x_k_hat = x_k_hat + K_k * (residual);
            
            %compute error covariance for updated estimate
            P_k = (eye(num_states) - K_k * H_temp) * P_k * ...
                (eye(num_states) - K_k * H_temp)' + ...
                K_k*R_k*K_k';
        end
        
%         %update estimate with measure z_k
%         prediction = h_x(x_k_hat);
%         %wrap the residual (don't ask why)
%         residual = z_k-prediction;
%         if(residual(7) < -pi)
%             residual(7) = residual(7) + 2*pi;
%         elseif(residual(7) > pi)
%             residual(7) = residual(7) - 2*pi;
%         end
%         x_k_hat = x_k_hat + K_k * (residual);
%         
%         %compute error covariance for updated estimate
%         P_k = (eye(num_states) - K_k * H_temp) * P_k * ...
%             (eye(num_states) - K_k * H_temp)' + ...
%             K_k*R_k*K_k';
        
        %project ahead
        x_k_hat = f_x(x_k_hat, prop_dt);
        phi_jac = f_prime(x_k_hat, prop_dt);
        P_k = phi_jac * P_k * phi_jac' + Q_k;
        
        %construct noise vector with covariance Q_k
        mu = zeros(1,num_states);
        w_k = (mvnrnd(mu, Q_k)');
        %generating realization
        addition = f_x(x_k, prop_dt);
        %display(addition(1:2));
        x_k = addition + w_k;
        x_k(4) = 1;%this is cheating, yolo
        
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time = 0:prop_dt:(length-prop_dt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%x axis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);subplot(2,1,1);
hold on;

%plotting one realization
realization_to_plot = reshape(previous_x_k(1,1,:), length/prop_dt,1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 4);
measurement_to_plot = reshape(previous_z_k(1,1,:), length/prop_dt, 1);
measurement_ = plot(time, measurement_to_plot, 'c', 'linewidth', 2);


%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,1,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,1,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_ measurement_], 'Realization', '$\sqrt{P_{1,1}}$', '$\hat{X_k}$', 'Measurement');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Position (m)');
title('East axis');
%print(gcf,'-depsc', ['fig_1' extension]);

subplot(2,1,2); hold on;
error = realization_to_plot-estimate;
error_ = plot(time,error,'r','LineWidth',2);
error_upper_ = plot(time,E,'b','LineWidth',2);
error_lower_ = plot(time,-E,'b','LineWidth',2);
legend_handle = legend([error_ error_upper_], 'Error', '$\sqrt{P_{1,1}}$');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Error in Position (m)');
title('East axis');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%y axis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2); subplot(2,1,1); hold on;
realization_to_plot = reshape(previous_x_k(1,2,:), length/prop_dt,1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 4);
measurement_to_plot = reshape(previous_z_k(1,2,:), length/prop_dt, 1);
measurement_ = plot(time, measurement_to_plot, 'c', 'linewidth', 2);


%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,2,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,2,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_ measurement_], 'Realization', '$\sqrt{P_{1,1}}$', '$\hat{X_k}$', 'Measurement');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Position (m)');
title('North axis');
%print(gcf,'-depsc', ['fig_1' extension]);

subplot(2,1,2); hold on;
error = realization_to_plot-estimate;
error_ = plot(time,error,'r','LineWidth',2);
error_upper_ = plot(time,E,'b','LineWidth',2);
error_lower_ = plot(time,-E,'b','LineWidth',2);
legend_handle = legend([error_ error_upper_], 'Error', '$\sqrt{P_{1,1}}$');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Error in Position (m)');
title('North axis');

%%%%%%%%%%
%plot some x,y shit
%%%%%%%%%%

figure(3);
hold on;
%x = reshape(previous_z_k(1,1,:), length/prop_dt,1);
%y = reshape(previous_z_k(1,2,:), length/prop_dt,1);
%plot(x,y, 'b', 'linewidth', 2);

%plot gps
gps_x = reshape(previous_z_k(1,1,:), length/prop_dt,1);
gps_y = reshape(previous_z_k(1,2,:), length/prop_dt,1);
pole_x = reshape(previous_x_k(1,7,:), length/prop_dt,1);
pole_y = reshape(previous_x_k(1,8,:), length/prop_dt,1);
pole_ = scatter(pole_x, pole_y, 'r');
velocities = reshape(previous_z_k(1,3,:), length/prop_dt, 1);
theta = reshape(previous_z_k(1,4,:), length/prop_dt, 1);
corrected = reshape(previous_x_k_hat(1,3,:), length/prop_dt, 1);

for ii = 1:size(gps_x)
    u(ii) = (velocities(ii)*cos(theta(ii)));
    v(ii) = (velocities(ii)*sin(theta(ii)));
    u_corrected(ii) = (velocities(ii)*cos(corrected(ii)));
    v_corrected(ii) = (velocities(ii)*sin(corrected(ii)));
end
u = u'; v = v'; u_corrected = u_corrected'; v_corrected = v_corrected';
%gps_ = quiver(gps_x, gps_y, u, v, 'r');
%quiver(gps_x, gps_y, u_corrected, v_corrected, 'r');

%plot estimate
est_x = reshape(previous_x_k_hat(1,1,:), length/prop_dt,1);
est_y = reshape(previous_x_k_hat(1,2,:), length/prop_dt,1);
heading_ = quiver(est_x, est_y, u_corrected, v_corrected, 'r', 'linewidth', 1);
est_ = plot(est_x, est_y, 'b', 'linewidth', 2);



legend_handle = legend([est_ heading_ pole_], 'Position', 'Estimated Heading', 'Pole Position');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Easting (m)');
ylabel('Northing (m)');
print(gcf,'-depsc', ['fig_3' extension]);
%axis([-1.5 0.5 -0.25 1.5]);
print(gcf,'-depsc', ['fig_3_zoomed' extension]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%theta (x3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(4); subplot(2,1,1); hold on;
realization_to_plot = reshape(previous_x_k(1,3,:), length/prop_dt,1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 4);
measurement_to_plot = reshape(previous_z_k(1,4,:), length/prop_dt, 1);
measurement_ = plot(time, measurement_to_plot, 'c', 'linewidth', 2);


%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,3,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,3,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_ measurement_], 'Realization', '$\sqrt{P_{1,1}}$', '$\hat{X_k}$', 'Measurement');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Heading (rads)');
title('Heading');
%print(gcf,'-depsc', ['fig_1' extension]);

subplot(2,1,2); hold on;
error = realization_to_plot-estimate;
error_ = plot(time,error,'r','LineWidth',2);
error_upper_ = plot(time,E,'b','LineWidth',2);
error_lower_ = plot(time,-E,'b','LineWidth',2);
legend_handle = legend([error_ error_upper_], 'Error', '$\sqrt{P_{3,3}}$');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Error in Heading (rads)');
title('Heading');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%linear velocity (x4)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(5); subplot(2,1,1); hold on;
realization_to_plot = reshape(previous_x_k(1,4,:), length/prop_dt,1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 4);
measurement_to_plot = reshape(previous_z_k(1,3,:), length/prop_dt, 1);
measurement_ = plot(time, measurement_to_plot, 'c', 'linewidth', 2);


%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,4,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,4,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_ measurement_], 'Realization', '$\sqrt{P_{1,1}}$', '$\hat{X_k}$', 'Measurement');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Velocity(m/s)');
title('Linear Velocity');
%print(gcf,'-depsc', ['fig_1' extension]);

subplot(2,1,2); hold on;
error = realization_to_plot-estimate;
error_ = plot(time,error,'r','LineWidth',2);
error_upper_ = plot(time,E,'b','LineWidth',2);
error_lower_ = plot(time,-E,'b','LineWidth',2);
legend_handle = legend([error_ error_upper_], 'Error', '$\sqrt{P_{4,4}}$');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Error in Velocity (m/s)');
title('Linear Velocity');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%angular velocity (x5)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(6); subplot(2,1,1); hold on;
realization_to_plot = reshape(previous_x_k(1,5,:), length/prop_dt,1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 4);
measurement_to_plot = reshape(previous_z_k(1,5,:), length/prop_dt, 1);
measurement_ = plot(time, measurement_to_plot, 'c', 'linewidth', 2);


%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,5,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,5,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_ measurement_], 'Realization', '$\sqrt{P_{1,1}}$', '$\hat{X_k}$', 'Measurement');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Velocity(rad/s)');
title('Angular Velocity');
%print(gcf,'-depsc', ['fig_1' extension]);

subplot(2,1,2); hold on;
error = realization_to_plot-estimate;
error_ = plot(time,error,'r','LineWidth',2);
error_upper_ = plot(time,E,'b','LineWidth',2);
error_lower_ = plot(time,-E,'b','LineWidth',2);
legend_handle = legend([error_ error_upper_], 'Error', '$\sqrt{P_{5,5}}$');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Error in Angular Velocity (rad/s)');
title('Angular Velocity');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Bias
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(7); subplot(2,1,1); hold on;
realization_to_plot = reshape(previous_x_k(1,6,:), length/prop_dt,1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 4);
measurement_to_plot = reshape(previous_z_k(1,4,:), length/prop_dt, 1);
measurement_ = plot(time, measurement_to_plot, 'c', 'linewidth', 2);


%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,6,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,6,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_ measurement_], 'Realization', '$\sqrt{P_{1,1}}$', '$\hat{X_k}$', 'Measurement');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Bias (rads)');
title('Bias');
%print(gcf,'-depsc', ['fig_1' extension]);

subplot(2,1,2); hold on;
error = realization_to_plot-estimate;
error_ = plot(time,error,'r','LineWidth',2);
error_upper_ = plot(time,E,'b','LineWidth',2);
error_lower_ = plot(time,-E,'b','LineWidth',2);
legend_handle = legend([error_ error_upper_], 'Error', '$\sqrt{P_{6,6}}$');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Error in Bias (rads)');
title('Bias');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%pole x position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(8); hold on;
realization_to_plot = reshape(previous_x_k(1,7,:), length/prop_dt,1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 4);
measurement_to_plot = reshape(previous_z_k(1,6,:), length/prop_dt, 1);
measurement_ = plot(time, measurement_to_plot, 'c', 'linewidth', 2);


%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,7,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,7,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_ measurement_], 'Realization', '$\sqrt{P_{1,1}}$', '$\hat{X_k}$', 'Measurement');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Position (m)');
title('East Pole 1');
print(gcf,'-depsc', ['fig_1' extension]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%z6 measurement stuff
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(9); hold on;
x_pole_realization = reshape(previous_x_k(1,7,:), length/prop_dt,1);
y_pole_realization = reshape(previous_x_k(1,8,:), length/prop_dt,1);
gps_x = reshape(previous_x_k(1,1,:), length/prop_dt,1);
gps_y = reshape(previous_x_k(1,2,:), length/prop_dt,1);
realization_to_plot = sqrt((gps_y-y_pole_realization).^2+(gps_x-x_pole_realization).^2);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 4);
measurement_to_plot = reshape(previous_z_k(1,6,:), length/prop_dt, 1);
measurement_ = plot(time, measurement_to_plot, 'c', 'linewidth', 2);


%plot P value as error bars on estimate
% E = sqrt(reshape(previous_P_k(1,7,:), length/prop_dt, 1));
% estimate = reshape(previous_x_k_hat(1,7,:), length/prop_dt, 1);
% error_bar_ = errorbar(time,estimate, E, 'k');
% estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly

legend_handle = legend([measurement_ realization_], 'Measurement', 'Realization');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Distance (m)');
title('Distance from pole');
print(gcf,'-depsc', ['fig_1' extension]);

%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %theta
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(4);
% hold on;
%
% %plotting one realization
% % realization_to_plot = reshape(previous_x_k(1,3,:), length/prop_dt,1);
% % realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
% realization_to_plot = reshape(previous_z_k(1,3,:), length/prop_dt, 1);
% realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 6);
%
% %plot P value as error bars on estimate
% E = sqrt(reshape(previous_P_k(1,4,:), length/prop_dt, 1));
% estimate = reshape(previous_x_k_hat(1,4,:), length/prop_dt, 1);
% error_bar_ = errorbar(time,estimate, E, 'k');
% estimate_ = plot(time, estimate, 'r', 'linewidth', 2);
%
% %make plot less ugly
% legend_handle = legend([realization_ error_bar_ estimate_], 'Odom Vel.', '$\sqrt{P_{3,3}}$', '$\hat{X_k}$');
% set(legend_handle, 'Interpreter', 'latex');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% title('Linear Velocity');
% print(gcf,'-depsc', ['fig_4' extension]);
%
% % figure(6);
% % hold on;
% %
% % %plotting error between realization and KF estimate
% % error = estimate - realization_to_plot;
% % error_ = plot(time, error, 'b', 'linewidth', 2);
% % %plot P_k
% % error_P_ = plot(time, E, 'r', 'linewidth', 2);
% % plot(time, -E, 'r', 'linewidth', 2);
% % %make plot less ugly
% % legend_ = legend([error_ error_P_], '$\hat{X_k}$ error', '$\sqrt{P_{3,3}}$');
% % set(legend_, 'Interpreter', 'latex');
% % xlabel('Time (s)');
% % ylabel('Theta (rad)');
% % title('Theta error');
% % print(gcf,'-depsc', ['fig_6' extension]);
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %linear velocity
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(5);
% hold on;
%
% %plotting one realization
% % realization_to_plot = reshape(previous_x_k(1,4,:), length/prop_dt,1);
% % realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
% realization_to_plot = reshape(previous_z_k(1,4,:), length/prop_dt, 1);
% realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
%
% %plot P value as error bars on estimate
% E = sqrt(reshape(previous_P_k(1,3,:), length/prop_dt, 1));
% estimate = reshape(previous_x_k_hat(1,3,:), length/prop_dt, 1);
% error_bar_ = errorbar(time,estimate, E, 'k');
% estimate_ = plot(time, estimate, 'r', 'linewidth', 2);
%
% %make plot less ugly
% legend_handle = legend([realization_ error_bar_ estimate_], 'Measured $\theta$', '$\sqrt{P_{3,3}}$', '$\hat{X_k}$');
% set(legend_handle, 'Interpreter', 'latex');
% xlabel('Time (s)');
% ylabel('Theta (rad)');
% title('Heading');
% print(gcf,'-depsc', ['fig_5' extension]);
%
% % figure(6);
% % hold on;
% %
% % %plotting error between realization and KF estimate
% % error = estimate - realization_to_plot;
% % error_ = plot(time, error, 'b', 'linewidth', 2);
% % %plot P_k
% % error_P_ = plot(time, E, 'r', 'linewidth', 2);
% % plot(time, -E, 'r', 'linewidth', 2);
% % %make plot less ugly
% % legend_ = legend([error_ error_P_], '$\hat{X_k}$ error', '$\sqrt{P_{3,3}}$');
% % set(legend_, 'Interpreter', 'latex');
% % xlabel('Time (s)');
% % ylabel('Velocity (m/s)');
% % title('Velocity erro');
% % print(gcf,'-depsc', ['fig_6' extension]);
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %angular velocity
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(6);
% hold on;
%
% %plotting one realization
% % realization_to_plot = reshape(previous_x_k(1,5,:), length/prop_dt,1);
% % realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
% realization_to_plot = reshape(previous_z_k(1,5,:), length/prop_dt, 1);
% realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
%
% %plot P value as error bars on estimate
% E = sqrt(reshape(previous_P_k(1,5,:), length/prop_dt, 1));
% estimate = reshape(previous_x_k_hat(1,5,:), length/prop_dt, 1);
% error_bar_ = errorbar(time,estimate, E, 'k');
% estimate_ = plot(time, estimate, 'r', 'linewidth', 2);
%
% %make plot less ugly
% legend_handle = legend([realization_ error_bar_ estimate_], 'Angular Velocity', '$\sqrt{P_{3,3}}$', '$\hat{X_k}$');
% set(legend_handle, 'Interpreter', 'latex');
% xlabel('Time (s)');
% ylabel('Velocity (rad/s)');
% title('Angular Velocity');
% print(gcf,'-depsc', ['fig_5' extension]);
%
%
% %simulated linear velocity
% figure(7);
% hold on;
% realization_to_plot = reshape(previous_x_k(1,4,:), length/prop_dt,1);
% realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
%
% legend([realization_], 'Simluated Linear Vel.');
% xlabel('Time(s)');
% ylabel('Velocity (m/s)');
%
% %plot theta bias
% figure(8);
% hold on;
% E = sqrt(reshape(previous_P_k(1,6,:), length/prop_dt, 1));
% estimate = reshape(previous_x_k_hat(1,6,:), length/prop_dt, 1);
% error_bar_ = errorbar(time,estimate, E, 'k');
% estimate_ = plot(time, estimate, 'r', 'linewidth', 2);
%
% legend([estimate_ error_bar_], 'Estimate', 'Error Bar');
% xlabel('Time(s)');
% ylabel('Bias (rad)');
%
% % figure(10);
% % hold on;
% %
% % %plotting error between realization and KF estimate
% % error = estimate - realization_to_plot;
% % error_ = plot(time, error, 'b', 'linewidth', 2);
% % %plot P_k
% % error_P_ = plot(time, E, 'r', 'linewidth', 2);
% % plot(time, -E, 'r', 'linewidth', 2);
% % %make plot less ugly
% % legend_ = legend([error_ error_P_], '$\hat{X_k}$ error', '$\sqrt{P_{3,3}}$');
% % set(legend_, 'Interpreter', 'latex');
% % xlabel('Time (s)');
% % ylabel('Velocity (rad/s)');
% % title('Angular velocity error');
% % print(gcf,'-depsc', ['fig_6' extension]);
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %theta bias
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % figure(11);
% % hold on;
% %
% % %plotting one realization
% % realization_to_plot = reshape(previous_x_k(1,6,:), length/prop_dt,1);
% % realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
% %
% % %plot P value as error bars on estimate
% % E = sqrt(reshape(previous_P_k(1,6,:), length/prop_dt, 1));
% % estimate = reshape(previous_x_k_hat(1,6,:), length/prop_dt, 1);
% % error_bar_ = errorbar(time,estimate, E, 'k');
% % estimate_ = plot(time, estimate, 'r', 'linewidth', 2);
% %
% % %make plot less ugly
% % legend_handle = legend([realization_ error_bar_ estimate_], 'Realization', '$\sqrt{P_{3,3}}$', '$\hat{X_k}$');
% % set(legend_handle, 'Interpreter', 'latex');
% % xlabel('Time (s)');
% % ylabel('Theta Bias (rad)');
% % title('Theta Bias');
% % print(gcf,'-depsc', ['fig_5' extension]);
% %
% % figure(12);
% % hold on;
% %
% % %plotting error between realization and KF estimate
% % error = estimate - realization_to_plot;
% % error_ = plot(time, error, 'b', 'linewidth', 2);
% % %plot P_k
% % error_P_ = plot(time, E, 'r', 'linewidth', 2);
% % plot(time, -E, 'r', 'linewidth', 2);
% % %make plot less ugly
% % legend_ = legend([error_ error_P_], '$\hat{X_k}$ error', '$\sqrt{P_{3,3}}$');
% % set(legend_, 'Interpreter', 'latex');
% % xlabel('Time (s)');
% % ylabel('Theta bias (m)');
% % title('Theta bias error');
% % print(gcf,'-depsc', ['fig_6' extension]);
%
%
