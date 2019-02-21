% the algorithms place the node randomly and the drone at the center of the map
% from the real distance, we create a perfect ESP with polynom
% we add a noise (+-noise_level) to this and consider we received that
% the drone moves in a direction and does nb_measures measures
% if it's better it continues, otherwise switches direction
% after a cycle, incremental distance divided by two and go back
% we consider the drone always flies at 10m

clear all; close all;

% define node coordinates xyz (z altitude)
node_position = [rand*200-100, rand*200-100, 0];
drone_position = [0, 0, 10];

% define increment in position
dist_increment = 10;
increment_0p0 = [0, dist_increment, 0];
increment_0m0 = [0, -dist_increment, 0];
increment_p00 = [dist_increment, 0, 0];
increment_m00 = [-dist_increment, 0, 0];

% init
time = 0; 
state = 0;
ESP = []; 
distance = [];

% load polynom
load('polynom_dist_to_ESP.mat', 'fitresult_dESP');
p_ESP_from_distance = fitresult_dESP;
load('polynom_ESP_to_dist.mat', 'fitresult_ESPd');
p_distance_from_ESP = fit_resultESPd;

% parameters
noise_level = 2; % +- 2dB
nb_measures_todo = 10;
time_limit = 1000;
pause_time = 0; % seconds

% localization
while time < time_limit
    
    % plot positions
    plot3(node_position(1), node_position(2), node_position(3), 'ro', 'MarkerSize', 10); grid on; hold on;
    if dist_increment == 10
        plot3(drone_position(1), drone_position(2), drone_position(3), 'bo', 'MarkerSize', 10);
    elseif dist_increment == 5
        plot3(drone_position(1), drone_position(2), drone_position(3), 'go', 'MarkerSize', 10);
    elseif dist_increment == 2.5
        plot3(drone_position(1), drone_position(2), drone_position(3), 'yo', 'MarkerSize', 10);
    elseif dist_increment == 1.25
        plot3(drone_position(1), drone_position(2), drone_position(3), 'ko', 'MarkerSize', 10);
    end
    view(0, 90);
    
    % recompute distance and get ESP (only temporary, will be from lora message afterwards
    distance = [distance, norm(drone_position - node_position)];
    perfect_ESP = ESP_from_distance(distance(end), p_ESP_from_distance);
    measured_ESP = perfect_ESP + rand()*2*noise_level - noise_level;
    ESP = [ESP, measured_ESP];
    
    switch state
        case 0 % starting point
            state = 1;
            
        case 1 % try p00 direction
            fprintf('Going p00 of %.1f meters\n', dist_increment);
            drone_position = drone_position + increment_p00;
            measures = 0;
            state = 2;
            
        case 2 % check progress
            measures = measures + 1;
            fprintf('#%d ', measures);
            if measures >= nb_measures_todo
                if mean(ESP(end-nb_measures_todo:end)) > ESP(end-nb_measures_todo-1) % better signal on average of last three
                    fprintf('\nProgress, continuing in this direction\n');
                    state = 1;
                else
                    fprintf('\nNo progress, go back then next direction\n');
                    drone_position = drone_position - increment_p00;
                    state = 3;
                end
            end
            
        case 3 % try m00 direction
            fprintf('Going m00 of %.1f meters\n', dist_increment);
            drone_position = drone_position + increment_m00;
            measures = 0;
            state = 4;
            
        case 4 %check progress
            measures = measures + 1;             
            fprintf('#%d ', measures);
            if measures >= nb_measures_todo
                if mean(ESP(end-nb_measures_todo:end)) > ESP(end-nb_measures_todo-1) % better signal on average of last three
                    fprintf('\nProgress, continuing in this direction\n');
                    state = 3;
                else
                    fprintf('\nNo progress, go back then next direction\n');
                    drone_position = drone_position - increment_m00;
                    state = 5;
                end
            end
            
        case 5 % try 0p0 direction
            fprintf('Going 0p0 of %.1f meters\n', dist_increment);
            drone_position = drone_position + increment_0p0;
            measures = 0;
            state = 6;
            
        case 6 % check progress
            measures = measures + 1;             
            fprintf('#%d ', measures);
            if measures >= nb_measures_todo
                if mean(ESP(end-nb_measures_todo:end)) > ESP(end-nb_measures_todo-1) % better signal on average of last three
                    fprintf('\nProgress, continuing in this direction\n');
                    state = 5;
                else
                    fprintf('\nNo progress, go back then next direction\n');
                    drone_position = drone_position - increment_0p0;
                    state = 7;
                end
            end
            
        case 7 % try m00 direction
            fprintf('Going 0m0 of %.1f meters\n', dist_increment);
            drone_position = drone_position + increment_0m0;
            measures = 0;
            state = 8;
            
        case 8 %check progress
            measures = measures + 1;             
            fprintf('#%d ', measures);
            if measures >= nb_measures_todo
                if mean(ESP(end-nb_measures_todo:end)) > ESP(end-nb_measures_todo-1) % better signal on average of last three
                    fprintf('\nProgress, continuing in this direction\n');
                    state = 7;
                else
                    fprintf('\nNo progress, go back then next direction\n');
                    drone_position = drone_position - increment_0m0;
                    state = 9;
                end
            end
            
        case 9
            % end condition (max one pass at 1.25m)
            if dist_increment < 1.5
                break;
            end
            
            % if reached maximum resolution for this increment, go smaller
            horizontal_dist = sqrt(max(distance_from_ESP(ESP(end), p_distance_from_ESP), 10)^2 - 100);
            fprintf('Estimated horizontal distance of %.2f m\n', horizontal_dist);
            if horizontal_dist < 2 * dist_increment
                dist_increment = dist_increment/2;
                increment_0p0 = [0, dist_increment, 0];
                increment_0m0 = [0, -dist_increment, 0];
                increment_p00 = [dist_increment, 0, 0];
                increment_m00 = [-dist_increment, 0, 0];
            end
            
            % go back to beginning of algorithm
            state = 1;
    end
    
    % time update
    time = time + 1;
    
    % slows down simulation
    pause(pause_time);
end

% estimated final position
estimated_position = drone_position;% + increment_p00 + increment_0p0; % increments only for without noise

% plot positions
plot3(node_position(1), node_position(2), node_position(3), 'ro', 'MarkerSize', 10); grid on; hold on;
plot3(drone_position(1), drone_position(2), drone_position(3), 'bo', 'MarkerSize', 10);
plot3(estimated_position(1), estimated_position(2), estimated_position(3), 'gx', 'MarkerSize', 10);
xlabel('x position [m]')
ylabel('y position [m]')
zlabel('z position [m]')
title('Node localization algorithm');
view(0, 90);

% print
fprintf('\nEstimated position of node: x=%.2f, y=%.2f\n', estimated_position(1), estimated_position(2));
fprintf('Real position of node: x=%.2f, y=%.2f\n', node_position(1), node_position(2));
fprintf('Error: dx=%.2f, dy=%.2f, norm=%.2f\n', abs(estimated_position(1) - node_position(1)), abs(estimated_position(2) - node_position(2)), norm([abs(drone_position(1) - node_position(1)), abs(drone_position(2) - node_position(2))])); 