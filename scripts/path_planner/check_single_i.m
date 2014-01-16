close all; clear all; clc;

waypoints = csvread('single_i_waypoints.txt');

boundary = [0, 0;
    15, 0;
    15, 4;
    0, 4;
    0, 0];
snowfield = [3, 1.5;
    13, 1.5;
    13, 2.5;
    3, 2.5;
    3, 1.5];
plot(boundary(:,1), boundary(:,2));
axis([-2 16 -6 10]);
hold on;
plot(snowfield(:,1), snowfield(:,2), 'g', 'LineWidth', 2);
plot(waypoints(:,1), waypoints(:,2), '-o');

%single i rotated
for ii = 1:size(boundary,1)
   boundary(ii,:) = rotation_matrix(boundary(ii,:),5);
end
for ii = 1:size(snowfield,1)
   snowfield(ii,:) = rotation_matrix(snowfield(ii,:),5); 
end
boundary

figure;
for ii = 1:length(waypoints(:,1))
    plot(boundary(:,1), boundary(:,2));
    axis([-20 20 -20 20]);
    hold on;
    plot(snowfield(:,1), snowfield(:,2), 'g', 'LineWidth', 2);
    plot(waypoints(1:ii,1), waypoints(1:ii,2), '-o');
    pause(.5);
    hold off;
end


%triple i
triple_i_waypoints = csvread('triple_i_waypoints.txt');

boundary = [0, 0;
    15, 0;
    15, 7;
    0, 7;
    0, 0];
snowfield = [3, 2;
    13, 2;
    13, 5;
    3, 5;
    3, 2];



%rotate boundary
for ii = 1:size(boundary,1)
   boundary(ii,:) = rotation_matrix(boundary(ii,:),4);
end
for ii = 1:size(snowfield,1)
   snowfield(ii,:) = rotation_matrix(snowfield(ii,:),4); 
end

figure;
for ii = 1:length(triple_i_waypoints(:,1))
    plot(boundary(:,1), boundary(:,2));
    axis([-20 20 -20 20]);
    hold on;
    plot(snowfield(:,1), snowfield(:,2), 'g', 'LineWidth', 2);
    plot(triple_i_waypoints(1:ii,1), triple_i_waypoints(1:ii,2), '-o');
    pause(.5);
    hold off;
end
