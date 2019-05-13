function distance = func_signal_to_distance_old(signal_strength)
% returns a distance from the signal stregnth (can be RSSI or ESP)

global func_a;
global func_b;

distance = func_a * exp(func_b * signal_strength);

end

