% the algorithms place the node randomly and the drone at the center of the map
% from the real distance, we create a perfect ESP with polynom
% we add a noise (+-noise_level) to this and consider we received that
% the drone moves in a direction and does nb_measures measures
% if it's better it continues, otherwise switches direction
% after a cycle, incremental distance divided by two and go back
% we consider the drone always flies at 10m

clear all; close all;

% init
time = 0; 
state = 0;

% parameters
plot_bool = true;
time_limit = 15*60;
time_period = 1;
altitude = 10;

% load function
load('func_ESP_to_distance.mat', 'fitresult_ESPd');
global func_a;
global func_b;
func_a = fitresult_ESPd.a;
func_b = fitresult_ESPd.b;

% define node coordinates xyz (z altitude)
node_position = [rand*200-100, rand*200-100, 0];
drone_position = [0, 0, altitude];
ESP = get_noisy_ESP(node_position, drone_position);

% define increment in position
dist_increment = 10;
increment_0p0 = [0, dist_increment, 0];
increment_0m0 = [0, -dist_increment, 0];
increment_p00 = [dist_increment, 0, 0];
increment_m00 = [-dist_increment, 0, 0];

% localization
while time < time_limit
    
    % plot positions
    if plot_bool
        plot_tri(node_position, 'ro'); grid on; hold on;
        if dist_increment == 10
            plot_tri(drone_position, 'bo');
        elseif dist_increment == 5
            plot_tri(drone_position, 'go');
        elseif dist_increment == 2.5
            plot_tri(drone_position, 'yo');
        else
            plot_tri(drone_position, 'ko');
        end
        view(0, 90); axis equal;
    end
    
    switch state
        case 0 % starting point
            state = 1;
            
        case 1 % try p00 direction
            fprintf('Going p00 of %.1f meters\n', dist_increment);
            drone_position = drone_position + increment_p00;
            state = 2;
            
        case 2 % make measures and check progress
            [ESP_new, ~] = get_noisy_ESP(node_position, drone_position);
            ESP = [ESP, ESP_new];
            if ESP(end) > ESP(end-1)    % better ESP
                fprintf('Progress, continuing in this direction\n');
                state = 1;
            else
                fprintf('No progress, go back then next direction\n');
                drone_position = drone_position - increment_p00;
                state = 3;
            end
            
        case 3 % try m00 direction
            fprintf('Going m00 of %.1f meters\n', dist_increment);
            drone_position = drone_position + increment_m00;
            state = 4;
            
        case 4 % make measures and check progress
            [ESP_new, ~] = get_noisy_ESP(node_position, drone_position);
            ESP = [ESP, ESP_new];
            if ESP(end) > ESP(end-1)    % better ESP
                fprintf('Progress, continuing in this direction\n');
                state = 3;
            else
                fprintf('No progress, go back then next direction\n');
                drone_position = drone_position - increment_m00;
                state = 5;
            end
            
        case 5 % try 0p0 direction
            fprintf('Going 0p0 of %.1f meters\n', dist_increment);
            drone_position = drone_position + increment_0p0;
            state = 6;
            
        case 6 % make measures and check progress
            [ESP_new, ~] = get_noisy_ESP(node_position, drone_position);
            ESP = [ESP, ESP_new];
            if ESP(end) > ESP(end-1)    % better ESP
                fprintf('Progress, continuing in this direction\n');
                state = 5;
            else
                fprintf('No progress, go back then next direction\n');
                drone_position = drone_position - increment_0p0;
                state = 7;
            end
            
        case 7 % try m00 direction
            fprintf('Going 0m0 of %.1f meters\n', dist_increment);
            drone_position = drone_position + increment_0m0;
            measures = 0;
            state = 8;
            
        case 8 % make measures and check progress
            [ESP_new, ~] = get_noisy_ESP(node_position, drone_position);
            ESP = [ESP, ESP_new];
            if ESP(end) > ESP(end-1)    % better ESP
                fprintf('Progress, continuing in this direction\n');
                state = 7;
            else
                fprintf('No progress, go back then next direction\n');
                drone_position = drone_position - increment_0m0;
                state = 9;
            end
            
        case 9
            % end condition (max one pass at 1.25m)
            if dist_increment < 1.5
                break;
            end
            
            % if reached maximum resolution for this increment, go smaller
            horizontal_dist = sqrt(max(func_signal_to_distance(ESP(end)), altitude)^2 - altitude^2);
            fprintf('Estimated horizontal distance of %.2f m\n', horizontal_dist);
            if horizontal_dist < 4 * dist_increment
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
    time = time + time_period;
    
    % slows down simulation
    pause(0.1);
end

% estimated final position
estimated_position = drone_position;

% plot positions
if plot_bool
    plot_tri(estimated_position, 'gx');
    xlabel('x position [m]')
    ylabel('y position [m]')
    zlabel('z position [m]')
    title('Node localization algorithm');
    view(0, 90); axis equal;
end

% print
fprintf('\nEstimated position of node: x=%.2f, y=%.2f\n', estimated_position(1), estimated_position(2));
fprintf('Real position of node: x=%.2f, y=%.2f\n', node_position(1), node_position(2));
fprintf('Error: dx=%.2f, dy=%.2f, norm=%.2f\n', abs(estimated_position(1) - node_position(1)), abs(estimated_position(2) - node_position(2)), norm([abs(drone_position(1) - node_position(1)), abs(drone_position(2) - node_position(2))])); 
fprintf('Found in t=%d loops\n', time);



% plot a vector of 3x1 in defined color
function plot_tri(vector, color)
    plot3(vector(1), vector(2), vector(3), color);
end

% obtain a noisy ESP and distance from positions
function [measured_ESP, measured_horizontal_distance] = get_noisy_ESP(node_position, measure_position)
    noise_level = 3;     % +-2dB
    number_measures = 3;
    
    ESP = zeros(number_measures,1);
    dist = zeros(number_measures,1);
    
    for i=1: number_measures
        real_dist = norm(measure_position - node_position);
        perfect_ESP = func_distance_to_signal(real_dist);
        ESP(i) = perfect_ESP + rand()*2*noise_level - noise_level;
        measured_distance = func_signal_to_distance(ESP(i));
        h = abs(node_position(3) - measure_position(3));
        measured_distance = max([measured_distance, h]);
        dist(i) = sqrt(measured_distance*measured_distance - h*h);
    end
    
    measured_ESP = mean(ESP);
    measured_horizontal_distance = mean(dist);
end