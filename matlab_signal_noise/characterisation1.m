%%
clear all;
close all;

% what we want:
% - distance and angle based on altitude with best signal and signal strength at this point
% - so we need 3D model of signal and noise
%
% todo:
% - find best angle alpha_optimal
%     - by using drone to collect data
%     - move up and down at fixed distance, get maximum signal
%     - antennas can stay straight for both
%     - can't do that yet (Ahmed still uses GW)
% - find typical values at alpha_optimal
%     - we will approach the node from this direction in the case of a 3D localization
%     - so we need to know distance as a function of signal INSIDE of this direction
%     - we can get one value from previous measures at 10m
%     - then two possibilities
%         - extrapolate based on previous results (same regression)
%         - make new measurements

% load polynom
load('polynom_dist_to_ESP.mat', 'fitresult_dESP');
p_dist_to_ESP = fitresult_dESP;

% dataset, Nx3 size, N points
% in order horizontal distance, vertical distance, signal strength
dataset = [20, 10, -92;
           50, 10, -102;
           100, 10, -112;
           150, 10, -119;
           200, 10, -126];

% define positions and arena
node_position = [0, 0, 0];
arena_size = 200;       % 200m in each direction
division_size = 20;      % 1m subdivisions
x_values = -arena_size:division_size:arena_size;
y_values = -arena_size:division_size:arena_size;
z_values = 0:division_size:arena_size;
ESP_values = zeros(length(x_values), length(y_values), length(z_values));

%%
% fills in ESP values
for i=1: length(x_values)
    for j=1: length(y_values)
        for k=1: length(z_values)
            horizontal_distance = norm([x_values(i), y_values(j)]);
            vertical_distance = z_values(k);
            ESP_values(i,j,k) = get_ESP(horizontal_distance, vertical_distance, p_dist_to_ESP);
        end
    end
end

%%
% plot
slice(x_values, y_values, z_values, ESP_values, [0], [0], [10]);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
colorbar;
colormap(jet);