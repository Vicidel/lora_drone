function distance = func_signal_to_distance(signal_strength, data_type)
% returns a distance from the signal stregnth (data_type can be 'esp' or 'rssi')

exp_a_esp = 0.1973;
exp_b_esp = -0.0902;
exp_a_rssi = 0.2189;
exp_b_rssi = -0.0894;

switch data_type
    case 'esp'
        distance = exp_a_esp * exp(exp_b_esp * signal_strength);
    case 'rssi'
        distance = exp_a_rssi * exp(exp_b_rssi * signal_strength);
    otherwise
        fprintf('Error, unknown type\n');
end

end

