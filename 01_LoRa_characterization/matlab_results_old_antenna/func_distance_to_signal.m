function signal_strength = func_distance_to_signal(distance, data_type)
% returns a signal stregnth (data_type can be 'esp' or 'rssi') from a distance

exp_a_esp = 0.2011;
exp_b_esp = -0.0556;
exp_a_rssi = 0.0273;
exp_b_rssi = -0.0777;

switch data_type
    case 'esp'
        signal_strength = log(distance/exp_a_esp) / exp_b_esp;
    case 'rssi'
        signal_strength = log(distance/exp_a_rssi) / exp_b_rssi;
    otherwise
        fprintf('Error, unknown type\n');
end

end

