function ESP = get_ESP(horizontal_distance, vertical_distance, p_dist_to_ESP)
% returns ESP based on hor and vert distances to antenna
% we consider the distances inside the antenna referential
   
% get base ESP from functions
ESP = ESP_from_distance(horizontal_distance, p_dist_to_ESP);

% attenuate with height
theta_deg = atan(vertical_distance/horizontal_distance)*180/pi;
coeff = signal_attenuation_angle(theta_deg);
ESP = ESP * coeff;

end

