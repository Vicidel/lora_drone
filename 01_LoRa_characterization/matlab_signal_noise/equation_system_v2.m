clear all; close all;

% load base params
func_a = 0.05568;
func_b = -0.1062;

% distances
distances = [20, 50, 100, 150, 200];
height = [10, 10, 10, 10, 10];
distances_hor = sqrt(distances.^2-height.^2);

% fill in attenuation for the different configurations and RSSI
attenuation_gateway = zeros(size(distances));
attenuation_node = zeros(size(distances));
RSSI = zeros(size(distances));
for i=1: length(distances)
    theta_deg = atan(height(i)/distances_hor(i))*180/pi;
    attenuation_gateway(i) = func_attenuation_angle(theta_deg)/2;
    attenuation_node(i) = func_attenuation_angle(theta_deg)/2;
    RSSI(i) = func_distance_to_signal(distances(i), 'rssi');
end


%%
% system
syms a b;
eqn1 = RSSI(2) == attenuation_gateway(2) + log(distances(2) / a) / b;
eqn5 = RSSI(5) == attenuation_gateway(5) + log(distances(5) / a) / b;
sol = vpasolve([eqn1, eqn5], [a, b], [func_a, func_b]);

% % store in mat file
exp_a = double(sol.a);
exp_b = double(sol.b);
save('matlab_results/coeff_RSSI_dist.mat', 'exp_a', 'exp_b');


%% 
% plot new a, b
figure();
fit_dist = 10:1:200;
fit_RSSI = log(fit_dist/sol.a) / sol.b;
fit_RSSI_old = log(fit_dist/func_a) / func_b;
plot(fit_dist, fit_RSSI); grid on; hold on;
plot(fit_dist, fit_RSSI_old);
xlabel('Distance [m]');
ylabel('RSSI [dB]');
legend('Decorrelated fit', 'Fit using data collected at h=10m');
title('RSSI attenuation due to angle');
