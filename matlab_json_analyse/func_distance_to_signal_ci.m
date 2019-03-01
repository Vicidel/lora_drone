function confidence_interval = func_distance_to_signal_ci(distance, ci)
% returns a signal stregnth confidence interval (can be RSSI or ESP) from a distance

a_min = ci(1,1);
a_max = ci(2,1);
b_min = ci(2,2);
b_max = ci(1,2);

signal_strength_min = log(distance / a_min) / b_min;
signal_strength_max = log(distance / a_max) / b_max;

confidence_interval = [signal_strength_max, signal_strength_min];

end

