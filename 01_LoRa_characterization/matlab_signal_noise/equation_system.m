clear all; close all;

% load function
load('coeff_RSSI_dist_old.mat', 'fitresult_RSSId');
global func_a;
global func_b;
func_a = fitresult_RSSId.a;
func_b = fitresult_RSSId.b;

% confidence interval
ci = confint(fitresult_RSSId);

% distances
distances = [20, 50, 100, 150, 200];
height = [10, 10, 10, 10, 10];
distances_hor = sqrt(distances.^2-height.^2);

% fill in attenuation for the different configurations and RSSI
attenuation_gateway_up = zeros(size(distances));
attenuation_node_up = zeros(size(distances));
RSSI_up_up = zeros(size(distances));
for i=1: length(distances)
    theta_deg = atan(height(i)/distances_hor(i))*180/pi;
    attenuation_gateway_up(i) = func_attenuation_angle(theta_deg);
    attenuation_node_up(i) = func_attenuation_angle(theta_deg);
    RSSI_up_up(i) = func_distance_to_signal(distances(i), 'rssi');
end


%%
% system
syms a b;
eqn1 = RSSI_up_up(1) == attenuation_gateway_up(1) + attenuation_node_up(1) + log(distances(1) / a) / b;
eqn2 = RSSI_up_up(2) == attenuation_gateway_up(2) + attenuation_node_up(2) + log(distances(2) / a) / b;
eqn3 = RSSI_up_up(3) == attenuation_gateway_up(3) + attenuation_node_up(3) + log(distances(3) / a) / b;
eqn4 = RSSI_up_up(4) == attenuation_gateway_up(4) + attenuation_node_up(4) + log(distances(4) / a) / b;
eqn5 = RSSI_up_up(5) == attenuation_gateway_up(5) + attenuation_node_up(5) + log(distances(5) / a) / b;
sol = vpasolve([eqn1, eqn5], [a, b], [func_a, func_b]);

% store in mat file
func_a_att = double(sol.a);
func_b_att = double(sol.b);
save('matlab_json_analyse/func_RSSI_to_dist_att.mat', 'func_a_att', 'func_b_att');


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
legend('New fit considering attenuation', 'Old fit');
title('RSSI attenuation due to angle');
