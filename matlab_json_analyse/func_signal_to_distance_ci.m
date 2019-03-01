function confidence_interval = func_signal_to_distance_ci(signal_strength, ci)
% returns a signal stregnth confidence interval (can be RSSI or ESP) from a distance

a_min = ci(1,1);
a_max = ci(2,1);
b_min = ci(2,2);
b_max = ci(1,2);

distance_min = a_min * exp(b_min * signal_strength);
distance_max = a_max * exp(b_max * signal_strength);

confidence_interval = [distance_min, distance_max];

end

