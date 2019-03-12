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
exp_a_att = 0.7056;
exp_b_att = 0.0388;

% get attenuation
db_attenuation = - exp_a_att * exp(exp_b_att*theta_deg);

% set values at 0 for 0
db_attenuation = db_attenuation + 0.7056;

end

