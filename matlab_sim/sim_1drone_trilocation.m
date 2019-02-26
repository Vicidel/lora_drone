% the algorithms place the node randomly and the drone at the center of the map
% from the real distance, we create a perfect ESP with polynom
% we add a noise (+-noise_level) to this and consider we received that
% the drone moves in three secion of map and takes three measures
% then trilocate the point

clear all; close all;

% define node coordinates xyz (z altitude)
arena_size = 100;
node_position = [rand*2*arena_size-arena_size, rand*2*arena_size-arena_size, 0];
drone_position = [0, 0, 10];
measure_position1 = [rand*arena_size, rand*arena_size, 10];    % x > 0, y > 0
measure_position2 = [-rand*arena_size, rand*arena_size, 10];    % x < 0, y > 0
measure_position3 = [rand*2*arena_size-arena_size, rand*arena_size-arena_size, 10];    % y < 0
distance = 9999;

% init
time = 0; 
state = 0;
ESP = zeros(3, 1); 

% load polynom
load('polynom_dist_to_ESP.mat', 'fitresult_dESP');
p_ESP_from_distance = fitresult_dESP;
load('polynom_ESP_to_dist.mat', 'fitresult_ESPd');
p_distance_from_ESP = fitresult_ESPd;

% parameters
noise_level = 3; % +- 2dB
time_limit = 1000;
pause_time = 0; % seconds
end_condition_distance = 10;
algo_loop = 0;
size_around_est = 40;
plot_bool = false;

% set first figure
if plot_bool 
    figure(1);
end

% localization
while time < time_limit
    
    if plot_bool
        % plot positions
        plot3(node_position(1), node_position(2), node_position(3), 'ko', 'MarkerSize', 10); grid on; hold on;
        plot3(drone_position(1), drone_position(2), drone_position(3), 'co', 'MarkerSize', 10);
        plot3(measure_position1(1), measure_position1(2), measure_position1(3), 'ro', 'MarkerSize', 10);
        plot3(measure_position2(1), measure_position2(2), measure_position2(3), 'go', 'MarkerSize', 10);
        plot3(measure_position3(1), measure_position3(2), measure_position3(3), 'bo', 'MarkerSize', 10);
        axis square; view(0, 90);
    end
    
    switch state
        case 0 % starting point
            delta = measure_position1 - drone_position;
            state = 1;
            
        case 1
            % move drone to position1
            [drone_position, next_state] = move_drone(drone_position, measure_position1);
            if next_state
                state = 2;
            end

        case 2
            % make a measure
            distance = norm(drone_position - node_position);
            perfect_ESP = ESP_from_distance(distance, p_ESP_from_distance);
            ESP_1st = perfect_ESP + rand()*2*noise_level - noise_level;
            ESP_2nd = perfect_ESP + rand()*2*noise_level - noise_level;
            ESP(1) = mean([ESP_1st, ESP_2nd]);
            delta = measure_position2 - drone_position;
            state = 3;

        case 3
            % move the drone in second position
            [drone_position, next_state] = move_drone(drone_position, measure_position2);
            if next_state
                state = 4;
            end
            
        case 4
            % make a measure
            distance = norm(drone_position - node_position);
            perfect_ESP = ESP_from_distance(distance, p_ESP_from_distance);
            ESP_1st = perfect_ESP + rand()*2*noise_level - noise_level;
            ESP_2nd = perfect_ESP + rand()*2*noise_level - noise_level;
            ESP(2) = mean([ESP_1st, ESP_2nd]);    
            delta = measure_position3 - drone_position;
            state = 5;

        case 5
            % move the drone in third position
            [drone_position, next_state] = move_drone(drone_position, measure_position3);
            if next_state
                state = 6;
            end
            
        case 6
            % make a measure
            distance = norm(drone_position - node_position);
            perfect_ESP = ESP_from_distance(distance, p_ESP_from_distance);
            ESP_1st = perfect_ESP + rand()*2*noise_level - noise_level;
            ESP_2nd = perfect_ESP + rand()*2*noise_level - noise_level;
            ESP(3) = mean([ESP_1st, ESP_2nd]);      
            state = 7;
            
        case 7
            % compute position
            [x, y] = get_position(measure_position1(1), measure_position1(2), distance_from_ESP(ESP(1), p_distance_from_ESP), ...
                                  measure_position2(1), measure_position2(2), distance_from_ESP(ESP(2), p_distance_from_ESP), ...
                                  measure_position3(1), measure_position3(2), distance_from_ESP(ESP(3), p_distance_from_ESP));
            estimated_position = [x, y, 10];
            if isnan(estimated_position(1)) || isnan(estimated_position(2))
                state = 0;
                if algo_loop == 0
                    fprintf('Could not find intersection, new random positions\n');
                    measure_position1 = [rand*100, rand*100, 10];    % x > 0, y > 0
                    measure_position2 = [-rand*100, rand*100, 10];    % x < 0, y > 0
                    measure_position3 = [rand*200-100, rand*100-100, 10];    % y < 0
                else
                    size_around_est = 60;
                    fprintf('Could not find intersection, larger spacing (%d m)\n', size_around_est);
                    measure_position1 = old_estimated_position + [0, -size_around_est, 0];   % south
                    measure_position2 = old_estimated_position + [size_around_est*cos(pi/6), size_around_est*sin(pi/6), 0];   % north east	
                    measure_position3 = old_estimated_position + [-size_around_est*cos(pi/6), size_around_est*sin(pi/6), 0];   % north west
                end
                close all;
            else
                fprintf('Found best intersection at x=%f and y=%f\n', x, y);
                
                if plot_bool
                    % plot positions and circles
                    figure();
                    plot3(node_position(1), node_position(2), node_position(3), 'ko', 'MarkerSize', 10); grid on; hold on;
                    plot3(measure_position1(1), measure_position1(2), measure_position1(3), 'ro', 'MarkerSize', 10);
                    plot3(measure_position2(1), measure_position2(2), measure_position2(3), 'go', 'MarkerSize', 10);
                    plot3(measure_position3(1), measure_position3(2), measure_position3(3), 'bo', 'MarkerSize', 10);
                    plot3(estimated_position(1), estimated_position(2), estimated_position(3), 'mx', 'MarkerSize', 10);
                    plot_circle(measure_position1(1), measure_position1(2), distance_from_ESP(ESP(1), p_distance_from_ESP), 'r');
                    plot_circle(measure_position2(1), measure_position2(2), distance_from_ESP(ESP(2), p_distance_from_ESP), 'g');
                    plot_circle(measure_position3(1), measure_position3(2), distance_from_ESP(ESP(3), p_distance_from_ESP), 'b');
                    xlabel('x position [m]')
                    ylabel('y position [m]')
                    zlabel('z position [m]')
                    title('Node localization algorithm');
                    legend('Node position', '1st measure', '2nd measure', '3rd measure', 'Estimated position');
                    view(0, 90); axis square; 
                end

                % print in terminal
                fprintf('Estimated position of node: x=%.2f, y=%.2f\n', estimated_position(1), estimated_position(2));
                fprintf('Real position of node: x=%.2f, y=%.2f\n', node_position(1), node_position(2));
                fprintf('Error: dx=%.2f, dy=%.2f, norm=%.2f\n', abs(estimated_position(1) - node_position(1)), abs(estimated_position(2) - node_position(2)), norm([abs(estimated_position(1) - node_position(1)), abs(estimated_position(2) - node_position(2))])); 
                
                % end condition
                if algo_loop == 1
                    fprintf('End of algorithm\n');
                    break;
                end
                
                % smaller size
                measure_position1 = estimated_position + [0, -size_around_est, 0];   % south
                measure_position2 = estimated_position + [size_around_est*cos(pi/6), size_around_est*sin(pi/6), 0];   % north east	
                measure_position3 = estimated_position + [-size_around_est*cos(pi/6), size_around_est*sin(pi/6), 0];   % north west
                
                % return to beginning
                fprintf('Restarting algorithm around found position\n\n');
                if plot_bool
                    figure();
                end
                state = 0;
                algo_loop = 1;
                old_estimated_position = estimated_position;
            end
            
    end
    
    % time update
    time = time + 1;
    
    % slows down simulation
    pause(pause_time);
end

fprintf('Found in t=%d loops\n', time);


% plots a circle in x, y, radius r in defined color 
function plot_circle(x, y, r, color)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    plot(xunit, yunit, color);
end

% obtain the position at intersection of three circles
function [xout, yout] = get_position(x1, y1, r1, x2, y2, r2, x3, y3, r3)

% get each duo of intersection position (could be NaN in no intersection --> test that)
[xout12, yout12] = circcirc(x1, y1, r1, x2, y2, r2);
[xout13, yout13] = circcirc(x1, y1, r1, x3, y3, r3);
[xout23, yout23] = circcirc(x3, y3, r3, x2, y2, r2);

% test if we have three intersection
if isnan(xout12(1)) || isnan(xout13(1)) || isnan(xout23(1)) || isnan(xout12(2)) || isnan(xout13(2)) || isnan(xout23(2))
    xout = NaN;
    yout = NaN;
    return;
end

% we don't, compute sum of distances between points
sum_vect = zeros(8, 1);
for i=1: 8
    if sum(i == [1, 2, 3, 4])   % test if we use first or second intersection for 12
        ind1 = 1;
    else 
        ind1 = 2;
    end
    if sum(i == [1, 2, 5, 6])   % test if we use first or second intersection for 13
        ind2 = 1;
    else 
        ind2 = 2;
    end
    if sum(i == [2, 4, 6, 8])   % test if we use first or second intersection for 23
        ind3 = 2;
    else 
        ind3 = 1;
    end
    
    % get sum of three distances between points
    sum_vect(i) = norm([xout12(ind1) - xout13(ind2), yout12(ind1) - yout13(ind2)]) + ...
                norm([xout12(ind1) - xout23(ind3), yout12(ind1) - yout23(ind3)]) + ...
                norm([xout23(ind3) - xout13(ind2), yout23(ind3) - yout13(ind2)]);
end

% find minimum and get corresponding indexes
[~, index] = min(sum_vect);
switch index
    case 1
        ind1 = 1; ind2 = 1; ind3 = 1;
    case 2
        ind1 = 1; ind2 = 1; ind3 = 2;
    case 3
        ind1 = 1; ind2 = 2; ind3 = 1;
    case 4
        ind1 = 1; ind2 = 2; ind3 = 2;
    case 5
        ind1 = 2; ind2 = 1; ind3 = 1;
    case 6
        ind1 = 2; ind2 = 1; ind3 = 2;
    case 7
        ind1 = 2; ind2 = 2; ind3 = 1;
    case 8
        ind1 = 2; ind2 = 2; ind3 = 2;
end

% return mean with computed indexes
xout = mean([xout12(ind1), xout13(ind2), xout23(ind3)]);
yout = mean([yout12(ind1), yout13(ind2), yout23(ind3)]);

end

% move drone from a to b
function [next_position, next_state] = move_drone(current_pos, goal)

max_dist = 10;
delta_pos = goal - current_pos;

if norm(delta_pos)>max_dist
    next_position = current_pos + max_dist * delta_pos/norm(delta_pos);
    next_state = false;
else
    next_position = goal;
    next_state = true;
end
 
end