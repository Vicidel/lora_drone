function coefficient = signal_attenuation_angle(theta_deg)
% get attenuation coefficient of the signal based on the angle theta

    theta = (theta_deg*pi/180) + pi/2;

    freq = 868*10^6;
    c = 299792458;
    lambda = c/freq;
    omega = 2*pi*freq;
    L_half = lambda/2;
    beta = 2*pi/L_half;
    E_theta_half = 1i * exp(1i * omega * (1)) * (cos(cos(theta) * beta * L_half / 2) - cos(beta * L_half / 2)) / (2 * sin(theta));
    
    coefficient = norm(E_theta_half);
    
end

