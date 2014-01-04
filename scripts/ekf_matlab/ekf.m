clear all; close all; clc;
extension = 'test';

%todo:
%fix the odometry angular velocities

%changed something in test 2
%change something in master after test3 branch
%change something in master after test3 branch round 2
%change something in master after test3 branch round 3
%changes that should try to break shit

real_data = csvread('real_robot_data/ekf_data_collect_2.txt');

%number of seconds
length = 50;

%number of realizations
num_realizations = 1;

%propogate dt (seconds)
prop_dt = .2;

%dynamics params
sigma_one = sqrt(.0025);%.0025
sigma_two = sqrt(.0025);%.0025
sigma_three = sqrt(.0001);%.0001
sigma_four = sqrt(.1);%.1
sigma_five = sqrt(.0625);%.0625
sigma_six = sqrt(.0001156);%.000156
beta_linear = .1;%.883
beta_angular = .1;%.883

%measurement params
sigma_z1 = sqrt(.0025);%.0025
sigma_z2 = sqrt(.0025);%.0025
sigma_z3 = sqrt(.0025);%.0025
sigma_z4 = sqrt(.0000076);%.0001
sigma_z5 = sqrt(.0025);%.0025

%save stuff to plot later
previous_x_k = zeros(num_realizations, 6, (length/prop_dt));
previous_z_k = zeros(num_realizations, 5, (length/prop_dt));
previous_x_k_hat = zeros(num_realizations, 6, (length/prop_dt));
previous_P_k = zeros(num_realizations, 6, (length/prop_dt));

%heading offset count (the number of times we need to multiply by 360)
heading_offset = 0;

for realization = 1:num_realizations
	%generate initial gyro bias
	gyro_bias = 1.57*randn(1);
	
	%set inital conditions (truth values)
	x_k = [	0;...
		0;...
		0;...%may want to change this to test gyro stuff
		0;...
		0;...
		gyro_bias];
	
	%set initial estimates
	x_k_hat = [	0;...%this won't be zero in actual implementation
		0;...%this won't be zero in actual implementation
		0;...
		0;...
		0;...
		0];
	
	%set initial estimate uncertainty
	P_k = zeros(6,6);
	P_k(1,1) = sigma_z1^2;
	P_k(2,2) = sigma_z2^2;
	P_k(3,3) = 1.57^2;
	P_k(4,4) = sigma_four^2;
	P_k(5,5) = sigma_five^2;
	P_k(6,6) = 1.57^2;
	
	
% 	phi/F
% 		f_x=@(x,dt)[x(1)+(sin(x(3))*x(4)*dt);...
% 					x(2)+(cos(x(3))*x(4)*dt);...
% 					x(3)+(x(5)*dt);...
% 					x(4)*exp(-beta_linear*dt);...
% 					x(5)*exp(-beta_angular*dt);...
% 					x(6)];
% 		f_prime=@(x,dt)[1 0 cos(x(3))*x(4)*dt sin(x(3))*dt 0 0;...
% 						0 1 -sin(x(3))*x(4)*dt cos(x(3))*dt 0 0;...
% 						0 0 1 0 dt 0;...
% 						0 0 0 exp(-beta_linear*dt) 0 0;...
% 						0 0 0 0 exp(-beta_angular*dt) 0;...
% 						0 0 0 0 0 1];
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
	h_x=@(x)[	x(1);...
		x(2);...
		x(4);...
		x(3)+x(6);...
		x(5)];
	H = [	1 0 0 0 0 0;...
		0 1 0 0 0 0;...
		0 0 0 1 0 0;...
		0 0 1 0 0 1;...
		0 0 0 0 1 0];
	
	%v matrix
	v_k = [	sigma_z1;
		sigma_z2;
		sigma_z3;
		sigma_z4;
		sigma_z5];
	
	%R matrix
	R_k = [ sigma_z1^2 0 0 0 0;...
		0 sigma_z2^2 0 0 0;...
		0 0 sigma_z3^2 0 0;...
		0 0 0 sigma_z4^2 0;...
		0 0 0 0 sigma_z5^2];
	
	%Q matrix
	Q_k = [ sigma_one^2 0 0 0 0 0;...
		0 sigma_two^2 0 0 0 0;...
		0 0 sigma_three^2 0 0 0;...
		0 0 0 sigma_four^2*(1-exp(-2*beta_linear*prop_dt)) 0 0;...
		0 0 0 0 sigma_five^2*(1-exp(-2*beta_angular*prop_dt)) 0;...
		0 0 0 0 0 sigma_six^2];
	
	for ii = 1:(length/prop_dt)
		%save realizations to plot later
		previous_x_k(realization, :, ii) = x_k;
		
		%plug in current estimate
		%H_temp = h_x(x_k_hat);
		
		%compute Kalman Gain
		K_k = P_k * H' * inv(H * P_k * H' + R_k);
		
		%simulate sensor measurements
		%z_k = H * x_k + v_k .*randn(5,1);
		z_k = real_data(ii,:);
		z_k = z_k';
		%check heading
		if(ii > 1)
			if(abs(real_data(ii,4)-real_data(ii-1,4)) > 3.1415926)
				display(x_k_hat);
				if(real_data(ii,4) > real_data(ii-1,4))
					heading_offset = heading_offset - 1;
				else
					heading_offset = heading_offset + 1;
				end
			end
		end
		z_k(4) = z_k(4) + heading_offset*2*3.1415926;
		
		
		%update estimate with measure z_k
		prediction = h_x(x_k_hat);			
		x_k_hat = x_k_hat + K_k * (z_k - prediction);
		if(ii > 1)
		if(abs(real_data(ii,4)-real_data(ii-1,4)) > 3.1415926)
			display(z_k);
			display(prediction);
			display(x_k_hat);
		end
		end
		
		%compute error covariance for updated estimate
		P_k = (eye(6) - K_k * H) * P_k;
		
		%save x_k_hat and error covariance
		previous_x_k_hat(realization, :, ii) = x_k_hat;
		previous_P_k(realization, :, ii) = diag(P_k);
		
		%project ahead
		x_k_hat = f_x(x_k_hat, prop_dt);
		phi_jac = f_prime(x_k_hat, prop_dt);
		P_k = phi_jac * P_k * phi_jac' + Q_k;
		
		%construct noise vector with covariance Q_k
		mu = zeros(1,6);
		w_k = (mvnrnd(mu, Q_k)');
		%generating realization
		addition = f_x(x_k, prop_dt);
		%display(addition(1:2));
		x_k = addition + w_k;
		
		%save measurements
		previous_z_k(realization, :, ii) = z_k;
		
	end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time = 0:prop_dt:(length-prop_dt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%x axis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);
hold on;

%plotting one realization
%realization_to_plot = reshape(previous_x_k(1,1,:), length/prop_dt,1);
%realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
realization_to_plot = reshape(previous_z_k(1,1,:), length/prop_dt, 1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 6);


%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,1,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,1,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_], 'GPS Measurement', '$\sqrt{P_{1,1}}$', '$\hat{X_k}$');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Position (m)');
title('X axis');
print(gcf,'-depsc', ['fig_1' extension]);

% figure(2);
% hold on;

% %plotting error between realization and KF estimate
% gps_error = reshape(previous_z_k(1,1,:), length/prop_dt,1) - realization_to_plot;
% gps_error_ = plot(time, gps_error, 'c', 'linewidth', 2);
% error = estimate - realization_to_plot;
% error_ = plot(time, error, 'b', 'linewidth', 2);
% %plot P_k
% error_P_ = plot(time, E, 'r', 'linewidth', 2);
% plot(time, -E, 'r', 'linewidth', 2);
% %make plot less ugly
% legend_ = legend([gps_error_ error_ error_P_], 'GPS error', '$\hat{X_k}$ error', '$\sqrt{P_{1,1}}$');
% set(legend_, 'Interpreter', 'latex');
% xlabel('Time (s)');
% ylabel('Position (m)');
% title('X axis error');
% print(gcf,'-depsc', ['fig_2' extension]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%y axis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2);
hold on;

%plotting one realization
% realization_to_plot = reshape(previous_x_k(1,2,:), length/prop_dt,1);
% realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
realization_to_plot = reshape(previous_z_k(1,2,:), length/prop_dt, 1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 6);

%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,2,:), length/prop_dt, 1));
% display(previous_x_k_hat(1,2,:));
estimate = reshape(previous_x_k_hat(1,2,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_], 'GPS Measurement', '$\sqrt{P_{2,2}}$', '$\hat{X_k}$');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Position (m)');
title('Y Axis');
print(gcf,'-depsc', ['fig_2' extension]);

% figure(4);
% hold on;

%plotting error between realization and KF estimate
% error = estimate - realization_to_plot;
% error_ = plot(time, error, 'b', 'linewidth', 2);
% %plot P_k
% error_P_ = plot(time, E, 'r', 'linewidth', 2);
% plot(time, -E, 'r', 'linewidth', 2);
% %make plot less ugly
% legend_ = legend([error_ error_P_], '$\hat{X_k}$ error', '$\sqrt{P_{2,2}}$');
% set(legend_, 'Interpreter', 'latex');
% xlabel('Time (s)');
% ylabel('Position (m)');
% title('Y Axis Error');
% print(gcf,'-depsc', ['fig_4' extension]);

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
%gps_ = plot(gps_x, gps_y, 'r', 'linewidth', 2);
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


legend_handle = legend([est_ heading_], 'Position', 'Estimated Heading');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Easting (m)');
ylabel('Northing (m)');
print(gcf,'-depsc', ['fig_3' extension]);
%axis([-1.5 0.5 -0.25 1.5]);
print(gcf,'-depsc', ['fig_3_zoomed' extension]);





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%theta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(4);
hold on;

%plotting one realization
% realization_to_plot = reshape(previous_x_k(1,3,:), length/prop_dt,1);
% realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
realization_to_plot = reshape(previous_z_k(1,3,:), length/prop_dt, 1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 6);

%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,4,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,4,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_], 'Odom Vel.', '$\sqrt{P_{3,3}}$', '$\hat{X_k}$');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Linear Velocity');
print(gcf,'-depsc', ['fig_4' extension]);

% figure(6);
% hold on;
%
% %plotting error between realization and KF estimate
% error = estimate - realization_to_plot;
% error_ = plot(time, error, 'b', 'linewidth', 2);
% %plot P_k
% error_P_ = plot(time, E, 'r', 'linewidth', 2);
% plot(time, -E, 'r', 'linewidth', 2);
% %make plot less ugly
% legend_ = legend([error_ error_P_], '$\hat{X_k}$ error', '$\sqrt{P_{3,3}}$');
% set(legend_, 'Interpreter', 'latex');
% xlabel('Time (s)');
% ylabel('Theta (rad)');
% title('Theta error');
% print(gcf,'-depsc', ['fig_6' extension]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%linear velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(5);
hold on;

%plotting one realization
% realization_to_plot = reshape(previous_x_k(1,4,:), length/prop_dt,1);
% realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
realization_to_plot = reshape(previous_z_k(1,4,:), length/prop_dt, 1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);

%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,3,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,3,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_], 'Measured $\theta$', '$\sqrt{P_{3,3}}$', '$\hat{X_k}$');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Theta (rad)');
title('Heading');
print(gcf,'-depsc', ['fig_5' extension]);

% figure(6);
% hold on;
%
% %plotting error between realization and KF estimate
% error = estimate - realization_to_plot;
% error_ = plot(time, error, 'b', 'linewidth', 2);
% %plot P_k
% error_P_ = plot(time, E, 'r', 'linewidth', 2);
% plot(time, -E, 'r', 'linewidth', 2);
% %make plot less ugly
% legend_ = legend([error_ error_P_], '$\hat{X_k}$ error', '$\sqrt{P_{3,3}}$');
% set(legend_, 'Interpreter', 'latex');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% title('Velocity erro');
% print(gcf,'-depsc', ['fig_6' extension]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%angular velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(6);
hold on;

%plotting one realization
% realization_to_plot = reshape(previous_x_k(1,5,:), length/prop_dt,1);
% realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
realization_to_plot = reshape(previous_z_k(1,5,:), length/prop_dt, 1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);

%plot P value as error bars on estimate
E = sqrt(reshape(previous_P_k(1,5,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,5,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

%make plot less ugly
legend_handle = legend([realization_ error_bar_ estimate_], 'Angular Velocity', '$\sqrt{P_{3,3}}$', '$\hat{X_k}$');
set(legend_handle, 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
title('Angular Velocity');
print(gcf,'-depsc', ['fig_5' extension]);


%simulated linear velocity
figure(7);
hold on;
realization_to_plot = reshape(previous_x_k(1,4,:), length/prop_dt,1);
realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);

legend([realization_], 'Simluated Linear Vel.');
xlabel('Time(s)');
ylabel('Velocity (m/s)');

%plot theta bias
figure(8);
hold on;
E = sqrt(reshape(previous_P_k(1,6,:), length/prop_dt, 1));
estimate = reshape(previous_x_k_hat(1,6,:), length/prop_dt, 1);
error_bar_ = errorbar(time,estimate, E, 'k');
estimate_ = plot(time, estimate, 'r', 'linewidth', 2);

legend([estimate_ error_bar_], 'Estimate', 'Error Bar');
xlabel('Time(s)');
ylabel('Bias (rad)');

% figure(10);
% hold on;
%
% %plotting error between realization and KF estimate
% error = estimate - realization_to_plot;
% error_ = plot(time, error, 'b', 'linewidth', 2);
% %plot P_k
% error_P_ = plot(time, E, 'r', 'linewidth', 2);
% plot(time, -E, 'r', 'linewidth', 2);
% %make plot less ugly
% legend_ = legend([error_ error_P_], '$\hat{X_k}$ error', '$\sqrt{P_{3,3}}$');
% set(legend_, 'Interpreter', 'latex');
% xlabel('Time (s)');
% ylabel('Velocity (rad/s)');
% title('Angular velocity error');
% print(gcf,'-depsc', ['fig_6' extension]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%theta bias
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(11);
% hold on;
%
% %plotting one realization
% realization_to_plot = reshape(previous_x_k(1,6,:), length/prop_dt,1);
% realization_ = plot(time, realization_to_plot, 'b', 'linewidth', 2);
%
% %plot P value as error bars on estimate
% E = sqrt(reshape(previous_P_k(1,6,:), length/prop_dt, 1));
% estimate = reshape(previous_x_k_hat(1,6,:), length/prop_dt, 1);
% error_bar_ = errorbar(time,estimate, E, 'k');
% estimate_ = plot(time, estimate, 'r', 'linewidth', 2);
%
% %make plot less ugly
% legend_handle = legend([realization_ error_bar_ estimate_], 'Realization', '$\sqrt{P_{3,3}}$', '$\hat{X_k}$');
% set(legend_handle, 'Interpreter', 'latex');
% xlabel('Time (s)');
% ylabel('Theta Bias (rad)');
% title('Theta Bias');
% print(gcf,'-depsc', ['fig_5' extension]);
%
% figure(12);
% hold on;
%
% %plotting error between realization and KF estimate
% error = estimate - realization_to_plot;
% error_ = plot(time, error, 'b', 'linewidth', 2);
% %plot P_k
% error_P_ = plot(time, E, 'r', 'linewidth', 2);
% plot(time, -E, 'r', 'linewidth', 2);
% %make plot less ugly
% legend_ = legend([error_ error_P_], '$\hat{X_k}$ error', '$\sqrt{P_{3,3}}$');
% set(legend_, 'Interpreter', 'latex');
% xlabel('Time (s)');
% ylabel('Theta bias (m)');
% title('Theta bias error');
% print(gcf,'-depsc', ['fig_6' extension]);


