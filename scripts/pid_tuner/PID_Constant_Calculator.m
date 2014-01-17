%-------------------------------------------------------------------------%
% Script Name:      PID_Constant_Calculator.m
% Programmer:       Ryan Wolfarth
% Inputs:           filename:   The name of the file containing heading and
%                               time information.
%                   Ku:         The ultimate proportional gain used in the 
%                               file above.
% Outputs:          Kp:         Proportional gain term for the PID
%                               algorithm.
%                   Ki:         Integral gain term for the PID algorithm.
%                   Kd:         Derivative gain term for the PID algorithm.
% Date Written:     2010?
% Date Modified:    19 December 2011
%
% This program calculates PID control constants based on the Ziegler-
% Nichols method. These constants are based on heading and time information
% from an input file and also on the ultimate proportional gain term  (Ku)
% that was used to generate said file.
%
% This program does not include frequency domain data in the stop-band of
% +/-0.1Hz when determining the ultimate period of oscillation (Tu).
%
%-------------------------------------------------------------------------%
clc;    clear all;  close all;

% Enter Ku used in this run
Ku = .45;
% Load the heading/time data
filename = 'pid_data.txt';
fp = fopen(filename);
if(~fp)
    error(['Error loading file: ' filename]);
end

% Read the entire file
data_cell = textscan(fp, '%f %f', 'Delimiter', ',');

% Assign time and heading variables
time = data_cell{1,1};
heading = data_cell{1,2};

% Close the file
fclose(fp);

% Adjust heading information
ind = heading > 180;
heading(ind) = heading(ind) - 360;

% Make time start at 0
time = (time - time(1));%/1000;

% Plot heading vs time
figure(1)
plot(time,heading);grid;
title('Heading vs. Time');
xlabel('Time (sec)');ylabel('Heading (degrees)');

% Setup variables for FFT
L = max(time);          % Length of data (s)
df = 1/L;               % Frequency division step size (Hz)
N = length(heading);    % Total number of points
f = -N/2:N/2-1;
f = f * df;             % Frequency axis vector (Hz)
sb = abs(f) < 0.1;      % Stop band: below 0.1Hz

% Take the FFT of the heading data
freq = fftshift(abs(fft(heading, N)));

% Plot spectrum
figure(2)
plot(f,freq);grid;
title('FFT: Heading vs. Time');
xlabel('Frequency (Hz)');ylabel('Magnitude');

% Fine the maximum value outside of the stop band
[val, ind] = max(freq(~sb));
Tu = abs(1/f(ind));

% Calculate and report PID control constants
Kp = 0.6*Ku
Ki = 2*Kp/Tu
Kd = Kp*Tu/8