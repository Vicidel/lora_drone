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
attenuation_gateway_up_db = 10*log(attenuation_gateway_up);
attenuation_node_up_db = 10*log(attenuation_node_up);

%%
% system
syms a c1 c2 c3;
eqn1 = ESP_up_up(1) == c1*attenuation_gateway_up_db(1) + c1*attenuation_node_up_db(1) - c3*log(distances(1) / a);
eqn2 = ESP_up_up(2) == c1*attenuation_gateway_up_db(2) + c1*attenuation_node_up_db(2) - c3*log(distances(2) / a);
eqn3 = ESP_up_up(3) == c1*attenuation_gateway_up_db(3) + c1*attenuation_node_up_db(3) - c3*log(distances(3) / a);
eqn4 = ESP_up_up(4) == c1*attenuation_gateway_up_db(4) + c1*attenuation_node_up_db(4) - c3*log(distances(4) / a);
eqn5 = ESP_up_up(5) == c1*attenuation_gateway_up_db(5) + c1*attenuation_node_up_db(5) - c3*log(distances(5) / a);
% eqn1 = ESP_up_up(1) == log(distances(1) / a) / b;
% eqn2 = ESP_up_up(2) == log(distances(2) / a) / b;
% eqn3 = ESP_up_up(3) == log(distances(3) / a) / b;
% eqn4 = ESP_up_up(4) == log(distances(4) / a) / b;
% eqn5 = ESP_up_up(5) == log(distances(5) / a) / b;
range = [func_a - 1, func_a + 1; -0.1, 0.1; 0, 100];
sol = vpasolve([eqn1, eqn2, eqn3], [a, c1, c3], range)
