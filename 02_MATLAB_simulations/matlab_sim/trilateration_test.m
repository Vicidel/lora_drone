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

% load function
load('func_ESP_to_distance.mat', 'fitresult_ESPd');
global func_a;
global func_b;
func_a = fitresult_ESPd.a;
func_b = fitresult_ESPd.b;

% get distances
[~, measured_distance_1] = get_noisy_ESP(node_position, drone_position_1);
[~, measured_distance_2] = get_noisy_ESP(node_position, drone_position_2);
[~, measured_distance_3] = get_noisy_ESP(node_position, drone_position_3);

% trilateration data
P = [drone_position_1' drone_position_2' drone_position_3'];
S = [measured_distance_1 measured_distance_2 measured_distance_3];
W = ones(size(S));

% trilateration
[x, y] = pylocus_get_position(P, S, W);

% plot 
figure();
plot_tri(node_position, 'ko'); grid on; hold on;
plot_tri(network_position, 'co');
plot_tri(drone_position_1, 'ro');
plot_tri(drone_position_2, 'go');
plot_tri(drone_position_3, 'bo');
plot3(x, y, 0, 'mx');
plot_circle(drone_position_1(1), drone_position_1(2), measured_distance_1, 'r');
plot_circle(drone_position_2(1), drone_position_2(2), measured_distance_2, 'g');
plot_circle(drone_position_3(1), drone_position_3(2), measured_distance_3, 'b');
axis equal; view(0, 90);
xlabel('x position [m]')
ylabel('y position [m]')
zlabel('z position [m]')
title('Localization with three drones - First measure');
legend('Real position', 'Network position', '1st drone', '2nd drone', '3rd drone', 'Estimated position');

a = 1;




% obtain the position of two intersection of three circles (in 3D)
% P is of size d.N with dimension d and N points, W weights (usually ones)
function [x, y] = pylocus_get_position(P, S, W)
    
    [d, N] = size(P(1:2, :));
    
    % for each point set A and b matrices
    A = []; b = [];
    for i=1: N
        x = P(1,i); y = P(2,i);
        s = S(i);
        A = [A ; -2*x  -2*y  1]; 
        b = [b ; s^2-x^2-y^2];
    end
    ATA = A.' * A;
    
    % set D and f
    D = zeros(d+1, d+1);
    D(1:d, 1:d) = eye(d);
    f = [zeros(1, d), -0.5]';
    
    % eigenvalues
    eigen = sort(eig(D, ATA));
    if eigen(end) > 1e-10
        lower_bound = -1/eigen(end);
    else
        fprintf('Warning: biggest eigenvalue is zero\n');
        lower_bound = -1e-5;
    end
    
    % parameters
    inf = 1e5; xtol = 1e-12;
    lambda_opt = 0;
    
    % find optimal lambda
    temp1 = pylocus_phi(lower_bound, ATA, D, A, b, f);
    temp2 = pylocus_phi(inf, ATA, D, A, b, f);
    if temp1 > 0 && temp2 < 0
        min_func = @(lambda) ((ATA + lambda*D)\(A*b - lambda*f))' * D * ((ATA + lambda*D)\(A*b - lambda*f)) + 2 * f' * ((ATA + lambda*D)\(A*b - lambda*f));
        %lambda_opt = fminsearch(min_func, lambda_opt);
        lambda_opt = fminbnd(min_func, lower_bound, inf);
    else
        fprintf('Lambda optimal set as 0...\n');
    end
    
    % get solution
    rhs = A*b - lambda_opt*f;
    lhs = ATA + lambda_opt*D;
    yhat = lhs\rhs;
    
    x = yhat(1);
    y = yhat(2);

end

% for pylocus
function yhat = pylocus_yhat(lambda, ATA, D, A, b, f)
    
    rhs = A*b - lambda*f;
    lhs = ATA + lambda*D;
    
    yhat = lhs\rhs;
end

% for pylocus
function phi = pylocus_phi(lambda, ATA, D, A, b, f)

    yhat = pylocus_yhat(lambda, ATA, D, A, b, f);
    phi = yhat' * D * yhat + 2 * f' * yhat;
    
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
function [measured_ESP, measured_horizontal_distance] = get_noisy_ESP(node_position, measure_position)
    noise_level = 3;     % +-2dB
    number_measures = 2;
    
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