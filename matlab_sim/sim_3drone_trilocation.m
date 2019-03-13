% the algorithms place the node at the origin
% then an estimated position is done (from network)
% the three drones are placed at equidistance around the estimation
% the drones then make a measure, then move towards it and remeasure
% estimation is obtainde from second estimation

clear all; close all;

% init 
global time_move;
global time_measure;
time_move = 0; 
time_measure = 0;
state = 0;

% simulation parameters
plot_bool = true;    % if true plot all
drone_speed = 1;     % m/s
time_limit = 60*20;  % battery limit
size_around_estimation_v1 = 70;     % size of triangle around estimation
size_around_estimation_v2 = 20;

% define node coordinates xyz (z altitude) and network estimation (circle of XXm)
node_position = [0, 0, 0];
network_error = 150;
phi = rand()*2*pi; rad = rand()*network_error;
network_position = node_position + [rad*cos(phi), rad*sin(phi), 0];

% localization
while time_move+time_measure < time_limit
    
    switch state
        case 0
            % fix the three drone positions
            drone_position_1_v1 = network_position + [0, -size_around_estimation_v1, 10];   % south
            drone_position_2_v1 = network_position + [size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 10];   % north east	
            drone_position_3_v1 = network_position + [-size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 10];   % north east	
            state = 1;
            
        case 1
            % move the drone, we consider teleportation possible
            time_move = time_move + size_around_estimation_v1/drone_speed;
            state = 2;
            
        case 2
            % make the three first measures  
            [measured_ESP_1, measured_distance_1] = get_noisy_ESP(node_position, drone_position_1_v1);
            [measured_ESP_2, measured_distance_2] = get_noisy_ESP(node_position, drone_position_2_v1);
            [measured_ESP_3, measured_distance_3] = get_noisy_ESP(node_position, drone_position_3_v1);

            % get estimation
            [x, y] = get_position(drone_position_1_v1(1), drone_position_1_v1(2), measured_distance_1, ...
                                  drone_position_2_v1(1), drone_position_2_v1(2), measured_distance_2, ...
                                  drone_position_3_v1(1), drone_position_3_v1(2), measured_distance_3);
            estimated_position_v1 = [x, y, 0];
            
            % check NaN
            if isnan(estimated_position_v1(1)) || isnan(estimated_position_v1(2))
                fprintf('First position found is NaN (no intersection), new measurements\n');
                state = 2;
            else
                % print
                fprintf('First position found at x=%.2f, y=%.2f\n', x, y);
                fprintf('Real position of node: x=%.2f, y=%.2f\n', node_position(1), node_position(2));
                fprintf('Error: dx=%.2f, dy=%.2f, norm=%.2f\n\n', abs(estimated_position_v1(1) - node_position(1)), abs(estimated_position_v1(2) - node_position(2)), norm([abs(estimated_position_v1(1) - node_position(1)), abs(estimated_position_v1(2) - node_position(2))])); 
            
                % plot
                if plot_bool
                    figure();
                    plot_tri(node_position, 'ko'); grid on; hold on;
                    plot_tri(network_position, 'co');
                    plot_tri(drone_position_1_v1, 'ro');
                    plot_tri(drone_position_2_v1, 'go');
                    plot_tri(drone_position_3_v1, 'bo');
                    plot_tri(estimated_position_v1, 'mx');
                    plot_circle(drone_position_1_v1(1), drone_position_1_v1(2), measured_distance_1, 'r');
                    plot_circle(drone_position_2_v1(1), drone_position_2_v1(2), measured_distance_2, 'g');
                    plot_circle(drone_position_3_v1(1), drone_position_3_v1(2), measured_distance_3, 'b');
                    axis equal; view(0, 90);
                    xlabel('x position [m]')
                    ylabel('y position [m]')
                    zlabel('z position [m]')
                    title('Localization with three drones - First measure');
                    legend('Real position', 'Network position', '1st drone', '2nd drone', '3rd drone', 'Estimated position');
                end
                state = 3;
            end
            
        case 3
            % new drone positions around estimation
            drone_position_1_v2 = estimated_position_v1 + [0, -size_around_estimation_v2, 10];   % south
            drone_position_2_v2 = estimated_position_v1 + [size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 10];   % north east	
            drone_position_3_v2 = estimated_position_v1 + [-size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 10];   % north east	
            state = 4;

        case 4
            % move the drone, we consider teleportation possible
            max_dist_to_cover = max([norm([drone_position_1_v2-drone_position_1_v1]), ...
                                    norm([drone_position_2_v2-drone_position_2_v1]), ...
                                    norm([drone_position_3_v2-drone_position_3_v1])]);
            time_move = time_move + max_dist_to_cover/drone_speed;
            state = 5;
            
        case 5
            % make the three second measures            
            [measured_ESP_1, measured_distance_1] = get_noisy_ESP(node_position, drone_position_1_v2);
            [measured_ESP_2, measured_distance_2] = get_noisy_ESP(node_position, drone_position_2_v2);
            [measured_ESP_3, measured_distance_3] = get_noisy_ESP(node_position, drone_position_3_v2);
            
            % get second estimation
            [x, y] = get_position(drone_position_1_v2(1), drone_position_1_v2(2), measured_distance_1, ...
                                  drone_position_2_v2(1), drone_position_2_v2(2), measured_distance_2, ...
                                  drone_position_3_v2(1), drone_position_3_v2(2), measured_distance_3);
            estimated_position_v2 = [x, y, 10];

            % check NaN
            if isnan(estimated_position_v2(1)) || isnan(estimated_position_v2(2))
                fprintf('Second position found is NaN (no intersection), new measurements\n');
                state = 5;
            else
                % print
                fprintf('Second position found at x=%.2f, y=%.2f\n', x, y);
                fprintf('Real position of node: x=%.2f, y=%.2f\n', node_position(1), node_position(2));
                fprintf('Error: dx=%.2f, dy=%.2f, norm=%.2f\n\n', abs(estimated_position_v2(1) - node_position(1)), abs(estimated_position_v2(2) - node_position(2)), norm([abs(estimated_position_v2(1) - node_position(1)), abs(estimated_position_v2(2) - node_position(2))])); 
                
                % plot
                if plot_bool
                    figure();
                    plot_tri(node_position, 'ko'); grid on; hold on;
                    plot_tri(drone_position_1_v2, 'ro');
                    plot_tri(drone_position_2_v2, 'go');
                    plot_tri(drone_position_3_v2, 'bo');
                    plot_tri(estimated_position_v2, 'mx');
                    plot_circle(drone_position_1_v2(1), drone_position_1_v2(2), measured_distance_1, 'r');
                    plot_circle(drone_position_2_v2(1), drone_position_2_v2(2), measured_distance_2, 'g');
                    plot_circle(drone_position_3_v2(1), drone_position_3_v2(2), measured_distance_3, 'b');
                    axis equal; view(0, 90);
                    xlabel('x position [m]')
                    ylabel('y position [m]')
                    zlabel('z position [m]')
                    title('Localization with three drones - Second measure');
                    legend('Real position', '1st drone', '2nd drone', '3rd drone', 'Estimated position');
                end
                
                fprintf('Found in t=%.1f seconds (%.1f moving and %.1f measuring)\n', time_move+time_measure, time_move, time_measure);
                
                % this is the end
                break; 
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