clear all;

% load function
load('func_ESP_to_distance.mat', 'fitresult_ESPd');
global func_a;
global func_b;
func_a = fitresult_ESPd.a;
func_b = fitresult_ESPd.b;

% distances
distances = [20, 50, 100, 150, 200];
height = [10, 10, 10, 10, 10];

% fill in attenuation for the different configurations
attenuation_gateway_up = zeros(size(distances));
attenuation_node_up = zeros(size(distances));
ESP_up_up = zeros(size(distances));
for i=1: length(distances)
    theta_deg = atan(height(i)/distances(i))*180/pi;
    attenuation_gateway_up(i) = signal_attenuation_angle(theta_deg);
    attenuation_node_up(i) = signal_attenuation_angle(theta_deg);
    ESP_up = func_distance_to_signal(distances(i));
end