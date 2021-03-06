function ESP = get_ESP(horizontal_distance, vertical_distance)
% returns ESP based on hor and vert distances to antenna
% we consider the distances inside the antenna referential
   
% get base ESP from functions
ESP = func_distance_to_signal(horizontal_distance, 'esp');

% attenuate with height
theta_deg = atan(vertical_distance/horizontal_distance)*180/pi;
attenuation_db = func_attenuation_angle(theta_deg);
ESP = ESP + attenuation_db;

end

