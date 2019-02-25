% the algorithms place the node at the origin
% then an estimated position is done (from network)
% the three drones are placed at equidistance around the estimation
% the drones then move closer and closer

clear all; close all;

% simulation parameters
plot_bool = true;   % if true plot all
noise_level = 2;    % +-2dB

% load polynom
load('polynom_dist_to_ESP.mat', 'fitresult_dESP');
p_ESP_from_distance = fitresult_dESP;
load('polynom_ESP_to_dist.mat', 'fitresult_ESPd');
p_distance_from_ESP = fitresult_ESPd;

% define node coordinates xyz (z altitude) and network estimation (circle of 200m)
node_position = [0, 0, 0];
network_error = 50;
phi = rand()*2*pi; rad = rand()*network_error;
network_position = node_position + [rad*cos(phi), rad*sin(phi), 0];

% fix the three drone positions
size_around_estimation = 70;
drone_position_1 = network_position + [0, -size_around_estimation, 0];   % south
drone_position_2 = network_position + [size_around_estimation*cos(pi/6), size_around_estimation*sin(pi/6), 0];   % north east	
drone_position_3 = network_position + [-size_around_estimation*cos(pi/6), size_around_estimation*sin(pi/6), 0];   % north east	

% plot base position
if plot_bool
    figure();
    plot_tri(node_position, 'ko'); grid on; hold on;
    plot_tri(network_position, 'co');
    plot_tri(drone_position_1, 'ro');
    plot_tri(drone_position_2, 'go');
    plot_tri(drone_position_3, 'bo');
    axis equal; view(0, 90);
    xlabel('x position [m]')
    ylabel('y position [m]')
    zlabel('z position [m]')
    title('Localization with three drones - Starting position');
    legend('Real position', 'Network position', '1st drone', '2nd drone', '3rd drone');
end

% make the three measures            
real_dist = norm(drone_position_1 - node_position);
perfect_ESP_1 = ESP_from_distance(real_dist, p_ESP_from_distance);
measured_ESP_1 = perfect_ESP_1 + rand()*2*noise_level - noise_level;
measured_distance_1 = distance_from_ESP(measured_ESP_1, p_distance_from_ESP);
real_dist = norm(drone_position_2 - node_position);
perfect_ESP_2 = ESP_from_distance(real_dist, p_ESP_from_distance);
measured_ESP_2 = perfect_ESP_2 + rand()*2*noise_level - noise_level;
measured_distance_2 = distance_from_ESP(measured_ESP_2, p_distance_from_ESP);
real_dist = norm(drone_position_3 - node_position);
perfect_ESP_3 = ESP_from_distance(real_dist, p_ESP_from_distance);
measured_ESP_3 = perfect_ESP_3 + rand()*2*noise_level - noise_level;
measured_distance_3 = distance_from_ESP(measured_ESP_3, p_distance_from_ESP);

% adds the circles
if plot_bool
    figure();
    plot_tri(node_position, 'ko'); grid on; hold on;
    plot_tri(network_position, 'co');
    plot_tri(drone_position_1, 'ro');
    plot_tri(drone_position_2, 'go');
    plot_tri(drone_position_3, 'bo');
    plot_circle(drone_position_1(1), drone_position_1(2), measured_distance_1, 'r');
    plot_circle(drone_position_2(1), drone_position_2(2), measured_distance_2, 'g');
    plot_circle(drone_position_3(1), drone_position_3(2), measured_distance_3, 'b');
    axis equal; view(0, 90);
    xlabel('x position [m]')
    ylabel('y position [m]')
    zlabel('z position [m]')
    title('Localization with three drones - First measure');
    legend('Real position', 'Network position', '1st drone', '2nd drone', '3rd drone');
end



% plot a vector of 3x1 in defined color
function plot_tri(vector, color)
    plot3(vector(1), vector(2), vector(3), color);
end

% plots a circle in x, y, radius r in defined color 
function plot_circle(x, y, r, color)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    plot(xunit, yunit, color);
end