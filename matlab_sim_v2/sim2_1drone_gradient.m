% follows a gradient of signal strength to find maximum
function output = sim2_1drone_gradient()

    clear all; close all;
    param;

    % init
    global time_move;
    global time_measure;
    time_move = 0; 
    time_measure = 0;
    state = 0;
    signal = [func_distance_to_signal(300, signal_type)];

    % create first figure
    if plot_bool
        figure();
        plot_tri(pos_true_node, 'ro'); grid on; hold on;
        plot_tri(pos_network_estimate, 'co');
        plot_tri(pos_drone, 'bo');
        plot_tri(pos_drone, 'mo');
        plot_tri(pos_drone, 'go');      % for legend purposes
        plot_tri(pos_drone, 'yo');
    end

    % drone takes off
    time_move = time_move + 20;
    pos_drone = pos_drone + [0, 0, altitude];


    % localization
    while time_move+time_measure < time_limit

        % plot positions
        if plot_bool
            if dist_increment == 10
                plot_tri(pos_drone, 'bo');
            elseif dist_increment == 5
                plot_tri(pos_drone, 'mo');
            elseif dist_increment == 2.5
                plot_tri(pos_drone, 'go');
            else
                plot_tri(pos_drone, 'yo');
            end
            view(0, 90); axis equal;
        end

        switch state
            case 0 % starting point
                state = 1;

            case 1 % try p00 direction
                pos_drone = pos_drone + increment_p00;
                time_move = time_move + norm(increment_p00)/drone_speed;
                state = 2;

            case 2 % make measures and check progress
                new_signal = get_noisy_signal(pos_true_node, pos_drone, signal_type, number_measures);
                signal = [signal, new_signal];
                if signal(end) > signal(end-1)    % better ESP
                    state = 1;
                else
                    pos_drone = pos_drone - increment_p00;
                    time_move = time_move + norm(increment_p00)/drone_speed;
                    state = 3;
                end

            case 3 % try m00 direction
                pos_drone = pos_drone + increment_m00;
                time_move = time_move + norm(increment_m00)/drone_speed;
                state = 4;

            case 4 % make measures and check progress
                new_signal = get_noisy_signal(pos_true_node, pos_drone, signal_type, number_measures);
                signal = [signal, new_signal];
                if signal(end) > signal(end-1)    % better ESP
                    state = 3;
                else
                    pos_drone = pos_drone - increment_m00;
                    time_move = time_move + norm(increment_m00)/drone_speed;
                    state = 5;
                end

            case 5 % try 0p0 direction
                pos_drone = pos_drone + increment_0p0;
                time_move = time_move + norm(increment_0p0)/drone_speed;
                state = 6;

            case 6 % make measures and check progress
                new_signal = get_noisy_signal(pos_true_node, pos_drone, signal_type, number_measures);
                signal = [signal, new_signal];
                if signal(end) > signal(end-1)    % better ESP
                    state = 5;
                else
                    pos_drone = pos_drone - increment_0p0;
                    time_move = time_move + norm(increment_0p0)/drone_speed;
                    state = 7;
                end

            case 7 % try m00 direction
                pos_drone = pos_drone + increment_0m0;
                time_move = time_move + norm(increment_p00)/drone_speed;
                state = 8;

            case 8 % make measures and check progress
                new_signal = get_noisy_signal(pos_true_node, pos_drone, signal_type, number_measures);
                signal = [signal, new_signal];
                if signal(end) > signal(end-1)    % better ESP
                    state = 7;
                else
                    pos_drone = pos_drone - increment_0m0;
                    time_move = time_move + norm(increment_0m0)/drone_speed;
                    state = 9;
                end

            case 9
                % end condition (max one pass at 1.25m)
                if dist_increment < 1.5
                    break;
                end

                % if reached maximum resolution for this increment, go smaller
                horizontal_dist = sqrt(max(func_signal_to_distance(signal(end), signal_type), altitude)^2 - altitude^2);
                if print_bool fprintf('Estimated horizontal distance of %.2f meters, time is %.1f seconds\n', horizontal_dist, time_move+time_measure); end
                if horizontal_dist < 6 * dist_increment
                    dist_increment = dist_increment/2;
                    if print_bool fprintf('Reducing increment to %.2f meters\n', dist_increment); end
                    increment_0p0 = [0, dist_increment, 0];
                    increment_0m0 = [0, -dist_increment, 0];
                    increment_p00 = [dist_increment, 0, 0];
                    increment_m00 = [-dist_increment, 0, 0];
                end

                % go back to beginning of algorithm
                state = 1;
        end
        
        % slows simulation
        pause(0.05);
    end

    % time limit reached
    if time_measure+time_move >= time_limit-2 && print_bool
        fprintf('\nTime limit reached (%.2f minutes passed)\n', (time_move+time_measure)/60);
    end

    % final estimated position
    pos_estimated = pos_drone;
    error_x = abs(pos_estimated(1) - pos_true_node(1));
    error_y = abs(pos_estimated(2) - pos_true_node(2));
    error_norm = norm([abs(error_x), abs(error_y)]);

    % plot positions
    if plot_bool
        plot_tri(pos_estimated, 'kx');
        xlabel('x position [m]')
        ylabel('y position [m]')
        zlabel('z position [m]')
        title('Node localization algorithm');
        legend('Node position', 'Network estimate', '10m increments', '5m increments', '2.5m increments', '1.25m increments');
        view(0, 90); axis equal;
    end

    % print
    if print_bool
        fprintf('\nRESULTS:\n');
        fprintf('Estimated position of node: x=%.2f, y=%.2f\n', pos_estimated(1), pos_estimated(2));
        fprintf('Real position of node: x=%.2f, y=%.2f\n', pos_true_node(1), pos_true_node(2));
        fprintf('Error: dx=%.2f, dy=%.2f, norm=%.2f\n', error_x, error_y, error_norm); 
        fprintf('Found in t=%.1f seconds (%.1f moving and %.1f measuring)\n', time_move+time_measure, time_move, time_measure);
    end
    
    % create output
    output.time_move_final = time_move;
    output.time_measure_final = time_measure;
    output.time_final = time_move + time_measure;
    output.final_precision = error_norm;
    output.pos_real = pos_true_node;
    output.pos_estimated = pos_estimated;
    
end



% plot a vector of 3x1 in defined color
function plot_tri(vector, color)
    plot3(vector(1), vector(2), vector(3), color);
end