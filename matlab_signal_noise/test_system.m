% now that we have a system that works, we can test it for different 
% antenna orientations: 45, up and horizontal

clear all; close all;

% load function
load('matlab_json_analyse\func_ESP_to_distance.mat', 'fitresult_ESPd');
global func_a;
global func_b;
func_a = fitresult_ESPd.a;
func_b = fitresult_ESPd.b;

% create the system
load('matlab_json_analyse\func_ESP_to_distance_att.mat');

% horizontal distances
distances_hor = [17, 49, 100, 150, 200];    % corresponds to our measuring distances
height = 10*ones(1, 5);

% fill in attenuation for the different configurations and ESP (from 0 to 1)
attenuation_gateway_up = zeros(size(distances_hor));
attenuation_gateway_45 = zeros(size(distances_hor));
attenuation_gateway_hor = zeros(size(distances_hor));
attenuation_node_up = zeros(size(distances_hor));
for i=1: length(distances_hor)
    theta_deg = atan(height(i)/distances_hor(i))*180/pi;
    attenuation_gateway_up(i) = signal_attenuation_angle(theta_deg);
    attenuation_gateway_45(i) = signal_attenuation_angle(theta_deg-45);
    attenuation_gateway_hor(i) = signal_attenuation_angle(theta_deg-90);
    attenuation_node_up(i) = signal_attenuation_angle(theta_deg);
end

% convert in dB
attenuation_gateway_up_db = 10*log(attenuation_gateway_up);
attenuation_gateway_45_db = 10*log(attenuation_gateway_45);
attenuation_gateway_hor_db = 10*log(attenuation_gateway_hor);
attenuation_node_up_db = 10*log(attenuation_node_up);

% measured values
ESP_up = -[92, 99, 105, 107, 120];
ESP_45 = -[82, 92, 99, 106, 120];
ESP_hor = -[92, 103, 102, 117, 120];

% problem here: we have values better at 45°, that doesn't make sense with
% our model. For us the angle of 45° would only worsen the signal, whch is
% not the case. So I don't know where to go from there..........

% create systems and solve
syms a b;
eqn1 = ESP_up(1) == attenuation_gateway_up_db(1) + attenuation_node_up_db(1) + log(distances_hor(1) / a) / b;
eqn4 = ESP_up(4) == attenuation_gateway_up_db(4) + attenuation_node_up_db(4) + log(distances_hor(4) / a) / b;
sol = vpasolve([eqn1, eqn4], [a, b], [func_a, func_b]);
func_a_up = double(sol.a); func_b_up = double(sol.b);
eqn1 = ESP_45(1) == attenuation_gateway_45_db(1) + attenuation_node_up_db(1) + log(distances_hor(1) / a) / b;
eqn4 = ESP_45(4) == attenuation_gateway_45_db(4) + attenuation_node_up_db(4) + log(distances_hor(4) / a) / b;
sol = vpasolve([eqn1, eqn4], [a, b], [func_a, func_b]);
func_a_45 = double(sol.a); func_b_45 = double(sol.b);
eqn1 = ESP_hor(1) == attenuation_gateway_hor_db(1) + attenuation_node_up_db(1) + log(distances_hor(1) / a) / b;
eqn4 = ESP_hor(4) == attenuation_gateway_hor_db(4) + attenuation_node_up_db(4) + log(distances_hor(4) / a) / b;
sol = vpasolve([eqn1, eqn4], [a, b], [func_a, func_b]);
func_a_hor = double(sol.a); func_b_hor = double(sol.b);

% plot
figure();
fit_dist = 10:1:200;
fit_ESP_up = log(fit_dist/func_a_up) / func_b_up;
fit_ESP_45 = log(fit_dist/func_a_45) / func_b_45;
plot(fit_dist, fit_ESP_up); grid on; hold on;
plot(fit_dist, fit_ESP_45);
xlabel('Distance [m]');
ylabel('ESP [dB]');
legend('Upward', '45°');
title('ESP attenuation due to angle');