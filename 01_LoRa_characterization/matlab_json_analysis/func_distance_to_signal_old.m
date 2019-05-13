function signal_strength = func_distance_to_signal_old(distance)
% returns a signal stregnth (can be RSSI or ESP) from a distance

global func_a;
global func_b;

signal_strength = log(distance / func_a) / func_b;

end

