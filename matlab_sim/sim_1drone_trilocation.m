% the algorithms place the node randomly and the drone at the center of the map
% from the real distance, we create a perfect ESP with polynom
% we add a noise (+-noise_level) to this and consider we received that
% the drone moves in three secion of map and takes three measures
% then trilocate the point

clear all; close all;

% init
global time_move;
global time_measure;
time_move = 0; 
time_measure = 0;
state = 0;
ESP = zeros(3, 1); 
algo_loop = 1;

% parameters
plot_bool = true;
algo_loops_todo = 2;
plot_movement_bool = false;
time_limit = 60*20;     % battery limit
drone_speed = 1;        % m/s
size_around_estimation_v1 = 100;    % size of triangle around estimation
size_around_estimation_v2 = 40;

% define node coordinates xyz (z altitude) and network estimation (circle of XXm)
node_position = [0, 0, 0];
network_error = 150;
phi = rand()*2*pi; rad = rand()*network_error;  
network_position = node_position + [rad*cos(phi), rad*sin(phi), 0];

% define measuring positions
measure_position1 = network_position + [0, -size_around_estimation_v1, 0];   % south
measure_position2 = network_position + [size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 0];   % north east	
measure_position3 = network_position + [-size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 0];   % north east	
            
% set drone starting point
drone_position = [rand*2*100-100, rand*2*100-100, 0];

% set first figure
if plot_movement_bool  && plot_bool
    figure(1);
end

% localization
while time_move+time_measure < time_limit
    
    if plot_movement_bool && plot_bool
        % plot movement
        plot_tri(node_position, 'ko'); grid on; hold on
        plot_tri(drone_position, 'co');
        plot_tri(measure_position1, 'ro');
        plot_tri(measure_position2, 'go');
        plot_tri(measure_position3, 'bo');
        axis equal; view(0, 90);
    end
    
    switch state
        case 0 % starting point
            delta = measure_position1 - drone_position;
            state = 1;
            
        case 1
            % move drone to position1
            [drone_position, next_state] = move_drone(drone_position, measure_position1);
            if next_state
                time_move = time_move + norm(delta)/drone_speed;
                state = 2;
            end

        case 2
            % make a measure
            [ESP(1), ~] = get_noisy_ESP(node_position, drone_position);
            delta = measure_position2 - drone_position;
            state = 3;

        case 3
            % move the drone in second position
            [drone_position, next_state] = move_drone(drone_position, measure_position2);
            if next_state
                time_move = time_move + norm(delta)/drone_speed;
                state = 4;
            end
            
        case 4
            % make a measure
            [ESP(2), ~] = get_noisy_ESP(node_position, drone_position);
            delta = measure_position3 - drone_position;
            state = 5;

        case 5
            % move the drone in third position
            [drone_position, next_state] = move_drone(drone_position, measure_position3);
            if next_state
                time_move = time_move + norm(delta)/drone_speed;
                state = 6;
            end
            
        case 6
            % make a measure
            [ESP(3), ~] = get_noisy_ESP(node_position, drone_position);
            state = 7;
            
        case 7
            % compute position
            [x, y] = get_position(measure_position1(1), measure_position1(2), func_signal_to_distance(ESP(1), 'esp'), ...
                                  measure_position2(1), measure_position2(2), func_signal_to_distance(ESP(2), 'esp'), ...
                                  measure_position3(1), measure_position3(2), func_signal_to_distance(ESP(3), 'esp'));
            estimated_position = [x, y, 10];
            
            % check intersection
            if isnan(estimated_position(1)) || isnan(estimated_position(2))
                state = 0;
                if algo_loop == 0
                    fprintf('Could not find intersection, new random positions\n');
                    measure_position1 = [rand*100, rand*100, 10];    % x > 0, y > 0
                    measure_position2 = [-rand*100, rand*100, 10];    % x < 0, y > 0
                    measure_position3 = [rand*200-100, rand*100-100, 10];    % y < 0
                else
                    fprintf('Could not find intersection, larger spacing (%d m)\n', size_around_estimation_v2);
                    size_around_estimation_v2 = 60;
                    measure_position1 = old_estimated_position + [0, -size_around_estimation_v2, 0];   % south
                    measure_position2 = old_estimated_position + [size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 0];   % north east	
                    measure_position3 = old_estimated_position + [-size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 0];   % north west
                end
            else
                % plot positions and circles
                if plot_bool
                    figure();
                    plot_tri(node_position, 'ko'); grid on; hold on;
                    if algo_loop == 1
                        plot_tri(network_position, 'co'); 
                    end
                    plot_tri(measure_position1, 'ro');
                    plot_tri(measure_position2, 'go');
                    plot_tri(measure_position3, 'bo');
                    plot_tri(estimated_position, 'mx');
                    plot_circle(measure_position1(1), measure_position1(2), func_signal_to_distance(ESP(1), 'esp'), 'r');
                    plot_circle(measure_position2(1), measure_position2(2), func_signal_to_distance(ESP(2), 'esp'), 'g');
                    plot_circle(measure_position3(1), measure_position3(2), func_signal_to_distance(ESP(3), 'esp'), 'b');
                    xlabel('x position [m]')
                    ylabel('y position [m]')
                    zlabel('z position [m]')
                    title('Node localization algorithm');
                    legend('Node position', 'Network position', '1st measure', '2nd measure', '3rd measure', 'Estimated position');
                    view(0, 90); axis equal; 
                end

                % print in terminal
                fprintf('Estimated position of node: x=%.2f, y=%.2f\n', estimated_position(1), estimated_position(2));
                fprintf('Real position of node: x=%.2f, y=%.2f\n', node_position(1), node_position(2));
                fprintf('Error: dx=%.2f, dy=%.2f, norm=%.2f\n', abs(estimated_position(1) - node_position(1)), abs(estimated_position(2) - node_position(2)), norm([abs(estimated_position(1) - node_position(1)), abs(estimated_position(2) - node_position(2))])); 
                
                % end condition
                if algo_loop == algo_loops_todo
                    fprintf('End of algorithm\n');
                    fprintf('Found in t=%.1f seconds (%.1f moving and %.1f measuring)\n', time_move+time_measure, time_move, time_measure);
                    break;
                end
                
                % smaller size
                measure_position1 = estimated_position + [0, -size_around_estimation_v2, 0];   % south
                measure_position2 = estimated_position + [size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 0];   % north east	
                measure_position3 = estimated_position + [-size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 0];   % north west
                
                % return to beginning
                fprintf('Restarting algorithm around found position\n\n');
                if plot_movement_bool && plot_bool
                    figure();
                end
                state = 0;
                algo_loop = algo_loop + 1;
                old_estimated_position = estimated_position;
            end
            
    end
    
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

% obtain a noisy ESP and distance from positions
function [measured_ESP, measured_horizontal_distance] = get_noisy_ESP(node_position, measure_position)
    number_measures = 2;
    
    ESP = zeros(number_measures,1);
    dist = zeros(number_measures,1);
    
    for i=1: number_measures
        % get distance and angle
        distances = measure_position - node_position;
        distance_ground = sqrt(distances(1)^2+distances(2)^2);
        real_dist = norm(distances);
        theta_deg = atan(distances(3)/distance_ground)*180/pi;
        
        % get ESP
        ESP_distance = func_distance_to_signal(real_dist, 'esp');
        ESP_attenuation = func_attenuation_angle(theta_deg);
        ESP(i) = ESP_distance + ESP_attenuation + normrnd(0, 2.5);
        
        % get distance
        measured_distance = func_signal_to_distance(ESP(i), 'esp');
        h = abs(distances(3));
        measured_distance = max([measured_distance, h]);
        dist(i) = sqrt(measured_distance*measured_distance - h*h);
    end
    
    measured_ESP = mean(ESP);
    measured_horizontal_distance = mean(dist);
    
    global time_measure;
    time_measure = time_measure + 5*number_measures;
end