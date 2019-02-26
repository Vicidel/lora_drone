% the algorithms place the node at the origin
% then an estimated position is done (from network)
% the three drones are placed at equidistance around the estimation
% the drones then make a measure, then move towards it and remeasure
% estimation is obtainde from second estimation

clear all; close all;

% init 
time = 0;
state = 0;

% simulation parameters
plot_bool = true;    % if true plot all
time_period = 1;
time_limit = 60*15;  % battery limit
size_around_estimation_v1 = 70;     % size of triangle around estimation
size_around_estimation_v2 = 20;

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

% localization
while time < time_limit
    
    switch state
        case 0
            % fix the three drone positions
            drone_position_1 = network_position + [0, -size_around_estimation_v1, 0];   % south
            drone_position_2 = network_position + [size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 0];   % north east	
            drone_position_3 = network_position + [-size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 0];   % north east	
            state = 1;
            
        case 1
            % move the drone, we consider teleportation possible
            state = 2;
            
        case 2
            % make the three first measures  
            [measured_ESP_1, measured_distance_1] = get_noisy_ESP(node_position, drone_position_1, p_ESP_from_distance, p_distance_from_ESP);
            [measured_ESP_2, measured_distance_2] = get_noisy_ESP(node_position, drone_position_2, p_ESP_from_distance, p_distance_from_ESP);
            [measured_ESP_3, measured_distance_3] = get_noisy_ESP(node_position, drone_position_3, p_ESP_from_distance, p_distance_from_ESP);

            % get estimation
            [x, y] = get_position(drone_position_1(1), drone_position_1(2), measured_distance_1, ...
                                  drone_position_2(1), drone_position_2(2), measured_distance_2, ...
                                  drone_position_3(1), drone_position_3(2), measured_distance_3);
            estimated_position_v1 = [x, y, 10];
            
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
                    plot_tri(drone_position_1, 'ro');
                    plot_tri(drone_position_2, 'go');
                    plot_tri(drone_position_3, 'bo');
                    plot_tri(estimated_position_v1, 'mx');
                    plot_circle(drone_position_1(1), drone_position_1(2), measured_distance_1, 'r');
                    plot_circle(drone_position_2(1), drone_position_2(2), measured_distance_2, 'g');
                    plot_circle(drone_position_3(1), drone_position_3(2), measured_distance_3, 'b');
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
            drone_position_1 = estimated_position_v1 + [0, -size_around_estimation_v2, 0];   % south
            drone_position_2 = estimated_position_v1 + [size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 0];   % north east	
            drone_position_3 = estimated_position_v1 + [-size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 0];   % north east	
            state = 4;

        case 4
            % move the drone, we consider teleportation possible
            state = 5;
            
        case 5
            % make the three second measures            
            [measured_ESP_1, measured_distance_1] = get_noisy_ESP(node_position, drone_position_1, p_ESP_from_distance, p_distance_from_ESP);
            [measured_ESP_2, measured_distance_2] = get_noisy_ESP(node_position, drone_position_2, p_ESP_from_distance, p_distance_from_ESP);
            [measured_ESP_3, measured_distance_3] = get_noisy_ESP(node_position, drone_position_3, p_ESP_from_distance, p_distance_from_ESP);
            
            % get second estimation
            [x, y] = get_position(drone_position_1(1), drone_position_1(2), measured_distance_1, ...
                                  drone_position_2(1), drone_position_2(2), measured_distance_2, ...
                                  drone_position_3(1), drone_position_3(2), measured_distance_3);
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
                    plot_tri(drone_position_1, 'ro');
                    plot_tri(drone_position_2, 'go');
                    plot_tri(drone_position_3, 'bo');
                    plot_tri(estimated_position_v2, 'mx');
                    plot_circle(drone_position_1(1), drone_position_1(2), measured_distance_1, 'r');
                    plot_circle(drone_position_2(1), drone_position_2(2), measured_distance_2, 'g');
                    plot_circle(drone_position_3(1), drone_position_3(2), measured_distance_3, 'b');
                    axis equal; view(0, 90);
                    xlabel('x position [m]')
                    ylabel('y position [m]')
                    zlabel('z position [m]')
                    title('Localization with three drones - Second measure');
                    legend('Real position', '1st drone', '2nd drone', '3rd drone', 'Estimated position');
                end
                
                % this is the end
                break; 
            end

    end
    
    % increase time
    time = time + time_period;
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
function [measured_ESP, measured_horizontal_distance] = get_noisy_ESP(node_position, measure_position, p_ESP_from_distance, p_distance_from_ESP)
    noise_level = 3;     % +-2dB
    number_measures = 2;
    
    ESP = zeros(number_measures,1);
    dist = zeros(number_measures,1);
    
    for i=1: number_measures
        real_dist = norm(measure_position - node_position);
        perfect_ESP = ESP_from_distance(real_dist, p_ESP_from_distance);
        ESP(i) = perfect_ESP + rand()*2*noise_level - noise_level;
        measured_distance = distance_from_ESP(ESP(i), p_distance_from_ESP);
        h = abs(node_position(3) - measure_position(3));
        measured_distance = max([measured_distance, h]);
        dist(i) = sqrt(measured_distance*measured_distance - h*h);
    end
    
    measured_ESP = mean(ESP);
    measured_horizontal_distance = mean(dist);
end