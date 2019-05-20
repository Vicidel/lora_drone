% ------------------------------------------------------------------------
% testing file to test the free-space path loss and angle attenuation
% we define the axis as north in positive y and east as positive x
% ------------------------------------------------------------------------


%% INITIALIZATION
% clear and close
clear all;
close all;
clc;

% set the position of drone (gateway) and node 
drone_position = [100, 100, 10];
node_position = [0, 0, 0];

% set the antenna angles
drone_antenna_angle = 0;    % 0 for towards ground
drone_orientation = 0;      % 0 for north orientation
node_antenna = 0;           % 0 for towards sky


%% COMPUTE DISTANCE AND ANGLES
% get the distance from drone to node
distances = drone_position - node_position;
distance_norm = norm(distances);
fprintf('The distance between the two devices is %.2f meters \n', distance_norm');

% get the angles
% ((for now we consider that the two are vertical))
distance_ground = sqrt(distances(1)^2+distances(2)^2);
angle_rad = atan(distances(3)/distance_ground);
angle_deg = angle_rad*180/pi;
fprintf('The angle between them is %.2f degrees\n', angle_deg);


%% GET FREE-SPACE PATH LOSS
% get the true signal from the distance
ESP_true = func_distance_to_signal(distance_norm, 'esp');
fprintf('From the distance, we obtain a noise-free free-space path loss of %.2f dBm\n', ESP_true);

% add 2.5 dB noise
ESP_noisy = ESP_true + normrnd(0, 2.5);
fprintf('With added normal noise of std=2.5dB: %.2f dBm\n', ESP_noisy);


%% GET ANGLE ATTENUATIONS
attenuation_db = func_attenuation_angle(angle_deg);
fprintf('The attenuation due to the angle between them is %.2f dB\n', attenuation_db);


%% SUM THE TWO COMPONENTS
ESP_received = ESP_noisy + attenuation_db;
fprintf('The final simulated ESP is %.2f dBm\n', ESP_received);


% ------------------------------------------------------------------------
% all before was only to get an estimate of the ESP in Matlab
% now we need to use this estimate to get back the distance
% ------------------------------------------------------------------------


%% MODIFY THE DISTANCE ACCORDING TO SIGNAL
distance_estimated = func_signal_to_distance(ESP_received, 'esp');
fprintf('Estimated distance of %.2f meters\n', distance_estimated);
fprintf('Error from real distance: %.2f meters\n', abs(distance_estimated-distance_norm));