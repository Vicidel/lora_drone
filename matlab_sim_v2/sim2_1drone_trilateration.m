% goes in three positions to do trilateration
function output = sim2_1drone_trilateration()

    clear all; close all;
    param;

    % init
    global time_move;
    global time_measure;
    time_move = 0; 
    time_measure = 0;
    state = 0;
    signal = zeros(3,1);
    algo_loop = 1;

    % create first figure
    if plot_movement_bool  && plot_bool
        figure(1);
    end

    % drone takes off
    time_move = time_move + 20;
    pos_drone = pos_drone + [0, 0, altitude];

    
    % localization
    while time_move+time_measure < time_limit

        if plot_movement_bool && plot_bool
            % plot movement
            plot_tri(pos_true_node, 'ko'); grid on; hold on
            plot_tri(pos_drone, 'co');
            plot_tri(measure_position1, 'ro');
            plot_tri(measure_position2, 'go');
            plot_tri(measure_position3, 'bo');
            axis equal; view(0, 90);
        end

        switch state
            case 0 % starting point
                delta = measure_position1 - pos_drone;
                state = 1;

            case 1
                % move drone to position1
                [pos_drone, next_state] = move_drone(pos_drone, measure_position1);
                if next_state
                    time_move = time_move + norm(delta)/drone_speed;
                    state = 2;
                end

            case 2
                % make a measure
                signal(1) = get_noisy_signal(pos_true_node, pos_drone, signal_type, number_measures);
                delta = measure_position2 - pos_drone;
                state = 3;

            case 3
                % move the drone in second position
                [pos_drone, next_state] = move_drone(pos_drone, measure_position2);
                if next_state
                    time_move = time_move + norm(delta)/drone_speed;
                    state = 4;
                end

            case 4
                % make a measure
                signal(2) = get_noisy_signal(pos_true_node, pos_drone, signal_type, number_measures);
                delta = measure_position3 - pos_drone;
                state = 5;

            case 5
                % move the drone in third position
                [pos_drone, next_state] = move_drone(pos_drone, measure_position3);
                if next_state
                    time_move = time_move + norm(delta)/drone_speed;
                    state = 6;
                end

            case 6
                % make a measure
                signal(3) = get_noisy_signal(pos_true_node, pos_drone, signal_type, number_measures);
                state = 7;

            case 7
                % compute position
                [x, y] = get_position_tri(measure_position1(1), measure_position1(2), func_signal_to_distance(signal(1), signal_type), ...
                                      measure_position2(1), measure_position2(2), func_signal_to_distance(signal(2), signal_type), ...
                                      measure_position3(1), measure_position3(2), func_signal_to_distance(signal(3), signal_type));
                pos_estimated = [x, y, 10];

                % check intersection
                if isnan(pos_estimated(1)) || isnan(pos_estimated(2))
                    state = 0;
                    if algo_loop == 1
                        if print_bool fprintf('Could not find intersection, new random positions\n'); end
                        measure_position1 = [rand*100, rand*100, 10];    % x > 0, y > 0
                        measure_position2 = [-rand*100, rand*100, 10];    % x < 0, y > 0
                        measure_position3 = [rand*200-100, rand*100-100, 10];    % y < 0
                    else
                        if print_bool fprintf('Could not find intersection, larger spacing (%d m)\n', size_around_estimation_v2); end
                        size_around_estimation_v2 = 60;
                        measure_position1 = pos_estimated_old + [0, -size_around_estimation_v2, 0];   % south
                        measure_position2 = pos_estimated_old + [size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 0];   % north east	
                        measure_position3 = pos_estimated_old + [-size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 0];   % north west
                    end
                else
                    % plot positions and circles
                    if plot_bool
                        figure();
                        plot_tri(pos_true_node, 'ko'); grid on; hold on;
                        if algo_loop == 1
                            plot_tri(pos_network_estimate, 'co'); 
                        end
                        plot_tri(measure_position1, 'ro');
                        plot_tri(measure_position2, 'go');
                        plot_tri(measure_position3, 'bo');
                        plot_tri(pos_estimated, 'mx');
                        plot_circle(measure_position1(1), measure_position1(2), func_signal_to_distance(signal(1), 'esp'), 'r');
                        plot_circle(measure_position2(1), measure_position2(2), func_signal_to_distance(signal(2), 'esp'), 'g');
                        plot_circle(measure_position3(1), measure_position3(2), func_signal_to_distance(signal(3), 'esp'), 'b');
                        xlabel('x position [m]')
                        ylabel('y position [m]')
                        zlabel('z position [m]')
                        title('Node localization algorithm');
                        legend('Node position', 'Network position', '1st measure', '2nd measure', '3rd measure', 'Estimated position');
                        view(0, 90); axis equal; 
                    end

                    % print in terminal
                    if print_bool
                        fprintf('Error after %d loop(s): dx=%.2f, dy=%.2f, norm=%.2f\n', algo_loop, abs(pos_estimated(1) - pos_true_node(1)), abs(pos_estimated(2) - pos_true_node(2)), norm([abs(pos_estimated(1) - pos_true_node(1)), abs(pos_estimated(2) - pos_true_node(2))])); 
                    end 
                    
                    % end condition
                    if algo_loop == algo_loops_todo 
                        break; 
                    end

                    % smaller size
                    measure_position1 = pos_estimated + [0, -size_around_estimation_v2, 0];   % south
                    measure_position2 = pos_estimated + [size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 0];   % north east	
                    measure_position3 = pos_estimated + [-size_around_estimation_v2*cos(pi/6), size_around_estimation_v2*sin(pi/6), 0];   % north west

                    % return to beginning
                    if print_bool fprintf('Restarting algorithm around found position\n\n'); end
                    if plot_movement_bool && plot_bool figure(); end
                    state = 0;
                    algo_loop = algo_loop + 1;
                    pos_estimated_old = pos_estimated;
                end
        end
    end
    
    % time limit reached
    if time_measure+time_move >= time_limit-2 && print_bool
        fprintf('\nTime limit reached (%.2f minutes passed)\n', (time_move+time_measure)/60);
    end
    
    % final estimated position
    error_x = abs(pos_estimated(1) - pos_true_node(1));
    error_y = abs(pos_estimated(2) - pos_true_node(2));
    error_norm = norm([abs(error_x), abs(error_y)]);

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

% plots a circle in x, y, radius r in defined color 
function plot_circle(x, y, r, color)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    plot(xunit, yunit, color);
end

% move drone from a to b
function [next_position, next_state] = move_drone(current_pos, goal)

    max_dist = 10;
    delta_pos = goal - current_pos;

    if norm(delta_pos)>max_dist
        next_position = current_pos + max_dist * delta_pos/norm(delta_pos);
        next_state = false;
    else
        next_position = goal;
        next_state = true;
    end

end
