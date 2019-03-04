clear all;

% load function
load('func_ESP_to_distance.mat', 'fitresult_ESPd');
global func_a;
global func_b;
func_a = fitresult_ESPd.a;
func_b = fitresult_ESPd.b;

% confidence interval
ci = confint(fitresult_ESPd);

% distances
distances = [20, 50, 100, 150, 200];
height = [10, 10, 10, 10, 10];
distances_hor = sqrt(distances.^2-height.^2);

% fill in attenuation for the different configurations and ESP
attenuation_gateway_up = zeros(size(distances));
attenuation_node_up = zeros(size(distances));
ESP_up_up = zeros(size(distances));
for i=1: length(distances)
    theta_deg = atan(height(i)/distances(i))*180/pi;
    attenuation_gateway_up(i) = signal_attenuation_angle(theta_deg);
    attenuation_node_up(i) = signal_attenuation_angle(theta_deg);
    ESP_up_up(i) = func_distance_to_signal(distances(i));
end
attenuation_gateway_up_db = 10*log(attenuation_gateway_up);
attenuation_node_up_db = 10*log(attenuation_node_up);


%%
% system
syms a b;
eqn1 = ESP_up_up(1) == attenuation_gateway_up_db(1) + attenuation_node_up_db(1) + log(distances(1) / a) / b;
eqn2 = ESP_up_up(2) == attenuation_gateway_up_db(2) + attenuation_node_up_db(2) + log(distances(2) / a) / b;
eqn3 = ESP_up_up(3) == attenuation_gateway_up_db(3) + attenuation_node_up_db(3) + log(distances(3) / a) / b;
eqn4 = ESP_up_up(4) == attenuation_gateway_up_db(4) + attenuation_node_up_db(4) + log(distances(4) / a) / b;
eqn5 = ESP_up_up(5) == attenuation_gateway_up_db(5) + attenuation_node_up_db(5) + log(distances(5) / a) / b;
sol = vpasolve([eqn1, eqn5], [a, b], [func_a, func_b]);

% store in mat file
func_a_att = double(sol.a);
func_b_att = double(sol.b);
save('matlab_json_analyse/func_ESP_to_distance_att.mat', 'func_a_att', 'func_b_att');


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
legend('New fit considering attenuation', 'Old fit');
title('ESP attenuation due to angle');
