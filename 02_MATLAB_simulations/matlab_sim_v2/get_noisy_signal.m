function [measured_signal] = get_noisy_signal(position1, position2, signal_type, number_measures)
% returns a signal based on multiple measures

    global time_measure;
    time_measure = time_measure + 5*number_measures;

    signal = zeros(number_measures,1);
    
    % get distance and angle
    distances = position1 - position2;
    distance_norm = norm(distances);
    distance_ground = sqrt(distances(1)^2+distances(2)^2);
    theta_deg = atan(distances(3)/distance_ground)*180/pi;
        
    for i=1: number_measures
        
        % get signal
        signal_distance = func_distance_to_signal(distance_norm, signal_type);
        signal_attenuation = func_attenuation_angle(theta_deg);
        signal(i) = signal_distance + signal_attenuation + normrnd(0, 2.5);     % adds gaussian noise of 2.5dB
    end
    
    % compute mean value
    measured_signal = mean(signal);
    
end

