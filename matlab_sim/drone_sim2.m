% the algorithms place the node randomly and the drone at the center of the map
% from the real distance, we create a perfect ESP with polynom
% we add a noise (+-noise_level) to this and consider we received that
% the drone moves in three secion of map and takes three measures
% then trilocate the point

clear all; close all;

% define node coordinates xyz (z altitude)
node_position = [rand*200-100, rand*200-100, 0];
drone_position = [0, 0, 10];
measure_position1 = [rand*100, rand*100, 0];    % x > 0, y > 0
measure_position2 = [-rand*100, rand*100, 0];    % x < 0, y > 0
measure_position3 = [rand*200-100, rand*100-100, 0];    % y < 0
distance = 9999;

% init
time = 0; 
state = 0;
ESP = zeros(3, 1); 

% load polynom
load('polynom_dist_to_ESP.mat', 'fitresult_dESP');
p_ESP_from_distance = fitresult_dESP;

% parameters
noise_level = 0; % +- 2dB
time_limit = 1000;
pause_time = 0; % seconds

% localization
while time < time_limit
    
%     % plot positions
%     plot3(node_position(1), node_position(2), node_position(3), 'ro', 'MarkerSize', 10); grid on; hold on;
%     plot3(drone_position(1), drone_position(2), drone_position(3), 'bo', 'MarkerSize', 10);
%     axis square; view(0, 90);
    
    switch state
        case 0 % starting point
            state = 1;
            
        case 1
            % move drone to position1
            drone_position = measure_position1;
            state = 2;

        case 2
            % make a measure
            distance = norm(drone_position - node_position);
            perfect_ESP = ESP_from_distance(distance, p_ESP_from_distance);
            ESP(1) = perfect_ESP + rand()*2*noise_level - noise_level;
            state = 3;

        case 3
            % move the drone in second position
            drone_position = measure_position2;
            state = 4;
            
        case 4
            % make a measure
            distance = norm(drone_position - node_position);
            perfect_ESP = ESP_from_distance(distance, p_ESP_from_distance);
            ESP(2) = perfect_ESP + rand()*2*noise_level - noise_level;
            state = 5;

        case 5
            % move the drone in third position
            drone_position = measure_position3;
            state = 6;
            
        case 6
            % make a measure
            distance = norm(drone_position - node_position);
            perfect_ESP = ESP_from_distance(distance, p_ESP_from_distance);
            ESP(3) = perfect_ESP + rand()*2*noise_level - noise_level;
            state = 7;
            
        case 7
            % compute position ((TODO))
            estimated_position = drone_position;
            break;
            
    end
    
    % time update
    time = time + 1;
    
    % slows down simulation
    pause(pause_time);
end


% plot positions and circles
plot3(node_position(1), node_position(2), node_position(3), 'ko', 'MarkerSize', 10); grid on; hold on;
plot3(measure_position1(1), measure_position1(2), measure_position1(3), 'ro', 'MarkerSize', 10);
plot3(measure_position2(1), measure_position2(2), measure_position2(3), 'go', 'MarkerSize', 10);
plot3(measure_position3(1), measure_position3(2), measure_position3(3), 'bo', 'MarkerSize', 10);
plot_circle(measure_position1(1), measure_position1(2), distance_from_ESP(ESP(1)), 'r');
plot_circle(measure_position2(1), measure_position2(2), distance_from_ESP(ESP(2)), 'g');
plot_circle(measure_position3(1), measure_position3(2), distance_from_ESP(ESP(3)), 'b');
xlabel('x position [m]')
ylabel('y position [m]')
zlabel('z position [m]')
title('Node localization algorithm');
legend('Node position', '1st measure', '2nd measure', '3rd measure');
view(0, 90); axis square; 

% print
fprintf('\nEstimated position of node: x=%.2f, y=%.2f\n', estimated_position(1), estimated_position(2));
fprintf('Real position of node: x=%.2f, y=%.2f\n', node_position(1), node_position(2));
fprintf('Error: dx=%.2f, dy=%.2f, norm=%.2f\n', abs(estimated_position(1) - node_position(1)), abs(estimated_position(2) - node_position(2)), norm([abs(drone_position(1) - node_position(1)), abs(drone_position(2) - node_position(2))])); 


function plot_circle(x, y, r, color)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    plot(xunit, yunit, color);
end
 