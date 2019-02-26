clear all; close all;

% fix the network node position 
node_position = [0, 0, 0];
network_error = 50;
phi = rand()*2*pi; rad = rand()*network_error;
network_position = node_position + [rad*cos(phi), rad*sin(phi), 0];

% fix the three drone locations
size_around_estimation_v1 = 70;
drone_position_1 = network_position + [0, -size_around_estimation_v1, 0];   % south
drone_position_2 = network_position + [size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 0];   % north east	
drone_position_3 = network_position + [-size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 0];   % north east	

% load polynom
load('polynom_dist_to_ESP.mat', 'fitresult_dESP');
p_ESP_from_distance = fitresult_dESP;
load('polynom_ESP_to_dist.mat', 'fitresult_ESPd');
p_distance_from_ESP = fitresult_ESPd;

% get distances
[~, measured_distance_1] = get_noisy_ESP(node_position, drone_position_1, p_ESP_from_distance, p_distance_from_ESP);
[~, measured_distance_2] = get_noisy_ESP(node_position, drone_position_2, p_ESP_from_distance, p_distance_from_ESP);
[~, measured_distance_3] = get_noisy_ESP(node_position, drone_position_3, p_ESP_from_distance, p_distance_from_ESP);

% trilateration data
P = [drone_position_1' drone_position_2' drone_position_3'];
S = [measured_distance_1 measured_distance_2 measured_distance_3];
W = ones(size(S));

% trilateration
[x, y] = get_position(P, S, W);

% plot 
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
legend('Real position', 'Network position', '1st drone', '2nd drone', '3rd drone', 'Estimated position');






% obtain the position of two intersection of three circles (in 3D)
% P is of size d.N with dimension d and N points, W weights (usually ones)
function [x, y] = get_position(P, S, W)
    
    [d, N] = size(P(1:2, :));

    % for each point set A and b matrices
    A = []; b = [];
    for i=1: N
        x = P(1,i); y = P(2,i);
        s = S(i);
        A = [A ; 1 -2*x  -2*y]; 
        b = [b ; s^2-x^2-y^2];
    end
    ATA = A.' * A;
    
    % set D and f
    D = zeros(d+1, d+1);
    D(1:d, 1:d) = eye(d);
    f = [zeros(1, d), -0.5]';
    
    % eigenvalues
    eigen = eig(D, ATA);
    if eigen(end) > 1e-10
        lower_bound = -1/eigen(end);
    else
        lower_bound = -1e-5;
    end
    
    % parameters
    inf = 1e5; xtol = 1e-12;
    lambda_opt = 0;
    
    % find optimal lambda
    'TODO'
    
    % solve ?
    lhs = ATA + lambda_opt*D;
    rhs = A*b - lambda_opt*f;
    y = lhs\rhs;
    
    
    x = 0;
    y = 0;

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