

function db_attenuation = func_attenuation_angle(theta_deg)
% get attenuation coefficient of the signal based on the angle theta

    angles = [0, 15, 30, 45, 60, 75, 90];
    attenuation = [3, 2, 1, -1, -5, -10, -1000] - 3;

    % to make it between 0 and 90 degrees
    if theta_deg < 0
        theta_deg = -theta_deg;
    end
    while theta_deg > 180
        theta_deg = theta_deg - 180;
    end
    if theta_deg > 90
        theta_deg = 180 -theta_deg;
    end
    
    if theta_deg == 90
        db_attenuation = NaN(1);
        return;
    end
    
    for i=1: 6
        if theta_deg >= angles(i) && theta_deg < angles(i+1)
            temp = 1-(theta_deg-angles(i))/15;
            db_attenuation = attenuation(i) * temp + attenuation(i+1) * (1-temp);
        end
    end
end

