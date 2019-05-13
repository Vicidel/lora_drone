clear all; close all;

% parameters
limit = 70;
node_position = [0, 0, 0];
x_values = -50:1:50;
y_values = -50:1:50;
altitude = 10;

% result
distance = zeros(length(x_values), length(y_values));
signal = zeros(length(x_values), length(y_values));

% fills result matrix
for i=1: length(x_values)
    for j=1: length(y_values)
        x = x_values(i);
        y = y_values(j);
        distance(i,j) = norm([x-node_position(1), y-node_position(2), altitude]);
        signal(i,j) = func_distance_to_signal(distance(i,j), 'esp');
    end
end

% plot
figure();
s = surf(y_values, x_values, signal);
s.EdgeColor = 'none';
xlabel('x [m]');
ylabel('y [m]');
zlabel('ESP [dBm]');
title('ESP gradient');
axis([-50 50 -50 50]);
view(0, 90);
colorbar;
