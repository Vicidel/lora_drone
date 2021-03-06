function db_attenuation = func_attenuation_angle(theta_deg)
% get attenuation coefficient of the signal based on the angle theta

% to make it between 0 and 90 degrees
if theta_deg < 0
    theta_deg = -theta_deg;
end
while theta_deg > 180
    theta_deg = theta_deg - 180;
end
if theta_deg > 90
    theta_deg = 180 - theta_deg;
end

% create exponential coefficients
a_att = 0.5667;
b_att = 1.38;

% get attenuation
db_attenuation = - a_att * theta_deg; % no term in b to have 0 at 0

end

