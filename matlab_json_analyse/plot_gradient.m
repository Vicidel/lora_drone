clear all; close all;

% parameters
limit = 50;
node_position = [rand*limit-limit/2, rand*limit-limit/2 0];
x_values = -limit:1:limit;
y_values = -limit:1:limit;
altitude = 10;

% load polynom
load('polynom_dist_to_ESP.mat', 'fitresult_dESP');
p_ESP_from_distance = fitresult_dESP;

% result
matrix = zeros(length(x_values), length(y_values));

% fills result matrix
for i=1: length(x_values)
    for j=1: length(y_values)
        x = x_values(i);
        y = y_values(j);
        distance = norm([x-node_position(1), y-node_position(2), altitude]);
        matrix(i, j) = ESP_from_distance(distance, p_ESP_from_distance);
    end
end

% plot
figure();
s = surf(x_values, y_values, matrix);
s.EdgeColor = 'none';
xlabel('x [m]');
ylabel('y [m]');
zlabel('ESP [dBm]');
title('ESP gradient');
view(0, 90);
colorbar;