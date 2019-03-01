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

%%
% system
syms Tx_0 a;
% a = func_a;
% b = func_b;
eqn1 = ESP_up_up(1) == attenuation_gateway_up(1) * attenuation_node_up(1) * Tx_0 * log(distances(1) / a);
eqn2 = ESP_up_up(2) == attenuation_gateway_up(2) * attenuation_node_up(2) * Tx_0 * log(distances(2) / a);
eqn3 = ESP_up_up(3) == attenuation_gateway_up(3) * attenuation_node_up(3) * Tx_0 * log(distances(3) / a);
eqn4 = ESP_up_up(4) == attenuation_gateway_up(4) * attenuation_node_up(4) * Tx_0 * log(distances(4) / a);
eqn5 = ESP_up_up(5) == attenuation_gateway_up(5) * attenuation_node_up(5) * Tx_0 * log(distances(5) / a);
sol = vpasolve([eqn1, eqn2, eqn3], [Tx_0, a], 'Random', true)