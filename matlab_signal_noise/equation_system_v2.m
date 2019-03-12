clear all; close all;

% load function
load('func_ESP_to_distance.mat', 'fitresult_ESPd');
global func_a;
global func_b;
func_a = fitresult_ESPd.a;
func_b = fitresult_ESPd.b;

% distances
distances = [20, 50, 100, 150, 200];
height = [10, 10, 10, 10, 10];
distances_hor = sqrt(distances.^2-height.^2);

% fill in attenuation for the different configurations and ESP
attenuation_gateway = zeros(size(distances));
attenuation_node = zeros(size(distances));
ESP = zeros(size(distances));
for i=1: length(distances)
    theta_deg = atan(height(i)/distances_hor(i))*180/pi;
    attenuation_gateway(i) = func_attenuation_angle(theta_deg);
    attenuation_node(i) = func_attenuation_angle(theta_deg);
    ESP(i) = func_distance_to_signal(distances(i));
end


%%
% system
syms a b;
eqn1 = ESP(1) == attenuation_gateway(1) + attenuation_node(1) + log(distances(1) / a) / b;
eqn5 = ESP(5) == attenuation_gateway(5) + attenuation_node(5) + log(distances(5) / a) / b;
sol = vpasolve([eqn1, eqn5], [a, b], [func_a, func_b]);

% % store in mat file
% exp_a = double(sol.a);
% exp_b = double(sol.b);
% save('matlab_results/func_ESP_dist.mat', 'exp_a', 'exp_b');


%% 
% plot new a, b
figure();
fit_dist = 10:1:200;
fit_ESP = log(fit_dist/sol.a) / sol.b;
fit_ESP_old = log(fit_dist/func_a) / func_b;
plot(fit_dist, fit_ESP); grid on; hold on;
plot(fit_dist, fit_ESP_old);
xlabel('Distance [m]');
ylabel('ESP [dB]');
legend('Decorrelated fit', 'Fit using data collected at h=10m');
title('ESP attenuation due to angle');
