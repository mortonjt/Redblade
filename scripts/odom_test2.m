


clear all;
close all;
filepath = 'gps_3_counterclockwise/';
clicks_per_m = 15768.6;

M = csvread([filepath 'out.front_encoder.csv']);

en_f.right = M(:,5);
en_f.left = M(:,4);
en_f.delta = M(:,3);
en_f.nsec = M(:,2);
en_f.sec = M(:,1);

M = csvread([filepath 'out.back_encoder.csv']);

en_b.right = M(:,5);
en_b.left = M(:,4);
en_b.delta = M(:,3);
en_b.nsec = M(:,2);
en_b.sec = M(:,1);

% get total time from encoder readings to temporarily use for cmd vel
% since it didn't get timestamped
total_time = round(en_f.sec(end) - en_f.sec(1));

% trim down encoder vectors to smallest size
en_len = min(length(en_f.right),length(en_b.right));
en_f.right = en_f.right(1:en_len);
en_l.left = en_f.left(1:en_len);
en_b.right = en_b.right(1:en_len);
en_b.left = en_b.left(1:en_len);


M = csvread([filepath 'out.front_cmd_vel.csv']);
vel_f.lin_x = M(:,1);
vel_f.ang_z = M(:,6);
num_samples = length(vel_f.lin_x);
vel_f.sec = total_time/num_samples * 1:num_samples;

M = csvread([filepath 'out.back_cmd_vel.csv']);
vel_b.lin_x = M(:,1);
vel_b.ang_z = M(:,6);
num_samples = length(vel_b.lin_x);
vel_b.sec = total_time/num_samples * 1:num_samples;

M = csvread([filepath 'out.gps.csv']);
gps.e = M(:,3);
gps.n = M(:,4);
gps.time = M(:,1) + M(:,2)*1e-9;


%% GPS velocities

diff_e = diff(gps.e);
diff_n = diff(gps.n);
diff_time = diff(gps.time);

gps_vel = (diff_e.^2 + diff_n.^2).^(1/2) ./ diff_time;

%% filter GPS vel
EMAorder = 20;
alpha = 2/(EMAorder  + 1);
coefficient = (1-alpha).^(1:EMAorder);
EMA_gps_vel = filter(coefficient, sum(coefficient), gps_vel);

%% Encoder velocities
delta_time_f = [0; diff(en_f.sec(1:en_len) + en_f.nsec(1:en_len)*1e-9)];
delta_time_b = [0; diff(en_b.sec(1:en_len) + en_b.nsec(1:en_len)*1e-9)];
% delta_time = (delta_time_f(1:en_len)+delta_time_b(1:en_len))/2;
diff_clicks_f = [0; diff(en_f.right) + diff(en_f.left)];
diff_clicks_b = [0; diff(en_b.right) + diff(en_b.left)];
en_clicks_r = (diff_clicks_f)./(2*delta_time_f*clicks_per_m);
en_clicks_l = (diff_clicks_b)./(2*delta_time_b*clicks_per_m);
en_vel = (en_clicks_r+en_clicks_l)./ 2;

%% find error
sample_ind = (0:length(en_vel)-1) ./ length(en_vel) * length(EMA_gps_vel);
testema = EMA_gps_vel(floor(sample_ind) + 1);
error = testema(32:end) - en_vel(1:end-31);

figure(3);
title('error between gps and encoder vel');
scatter(en_vel(1:end-31), error);

figure(2);
plot(testema(32:end))
hold on
plot(en_vel,'r')



