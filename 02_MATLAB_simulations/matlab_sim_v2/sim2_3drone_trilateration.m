% trilaterates using three drones, flies at 10m height
function output = sim2_3drone_trilateration()

    clear all; close all;
    param;

    % init
    global time_move;
    global time_measure;
    time_move = 0; 
    time_measure = 0;
    state = 0;
    signal = zeros(3,1);
    distances = zeros(3,1);
    algo_loop = 1;

    % drone takes off
    time_move = time_move + 20;
    pos_drone1 = pos_drone1 + [0, 0, altitude];
    pos_drone2 = pos_drone2 + [0, 0, altitude];
    pos_drone3 = pos_drone3 + [0, 0, altitude];

    
    % localization
    while time_move+time_measure < time_limit
    
        switch state
            case 0 % starting point
                state = 1;

            case 1
                % move the drone
                max_distance = max([abs(pos_drone1-measure_position1), abs(pos_drone2-measure_position2), abs(pos_drone3-measure_position3)]);
                time_move = time_move + max_distance/drone_speed;
                state = 2;

            case 2
                % make the three measures  
                signal(1) = get_noisy_signal(pos_true_node, measure_position1, signal_type, number_measures);
                signal(2) = get_noisy_signal(pos_true_node, measure_position2, signal_type, number_measures);
                signal(3) = get_noisy_signal(pos_true_node, measure_position3, signal_type, number_measures);
                distances(1) = func_signal_to_distance(signal(1), signal_type);
                distances(2) = func_signal_to_distance(signal(2), signal_type);
                distances(3) = func_signal_to_distance(signal(3), signal_type);

                % get estimation
                [x, y] = get_position_tri(measure_position1(1), measure_position1(2), distances(1), ...
                                      measure_position2(1), measure_position2(2), distances(2),...
                                      measure_position3(1), measure_position3(2), distances(3));
                pos_estimated = [x, y, 0];

                % check NaN
                if isnan(pos_estimated(1)) || isnan(pos_estimated(2))
                    if print_bool fprintf('Position found is NaN (no intersection), new measurements\n'); end
                    state = 2;
                else                    
                    % plot
                    if plot_bool
                        figure();
                        plot_tri(pos_true_node, 'ko'); grid on; hold on;
                        plot_tri(pos_network_estimate, 'co');
                        plot_tri(measure_position1, 'ro');
                        plot_tri(measure_position2, 'go');
                        plot_tri(measure_position3, 'bo');
                        plot_tri(pos_estimated, 'mx');
                        plot_circle(measure_position1(1), measure_position1(2), distances(1), 'r');
                        plot_circle(measure_position2(1), measure_position2(2), distances(2), 'g');
                        plot_circle(measure_position3(1), measure_position3(2), distances(3), 'b');
                        axis equal; view(0, 90);
                        xlabel('x position [m]')
                        ylabel('y position [m]')
                        zlabel('z position [m]')
                        title('Node localization algorithm');
                        legend('Real position', 'Network position', '1st drone', '2nd drone', '3rd drone', 'Estimated position');
                    end
                    
                    % print in terminal
                    if print_bool
                        fprintf('Error after %d loop(s): dx=%.2f, dy=%.2f, norm=%.2f\n', algo_loop, abs(pos_estimated(1) - pos_true_node(1)), abs(pos_estimated(2) - pos_true_node(2)), norm([abs(pos_estimated(1) - pos_true_node(1)), abs(pos_estimated(2) - pos_true_node(2))])); 
                        fprintf('Time passed: %.2f\n', time_move+time_measure);
                    end 
                    
                    % store inter
                    if algo_loop == 1
                        output.inter_time_move = time_move;
                        output.inter_time_measure = time_measure;
                        output.inter_time = time_move + time_measure;
                        output.inter_precision = norm([abs(abs(pos_estimated(1) - pos_true_node(1))), abs(abs(pos_estimated(2) - pos_true_node(2)))]);
                        output.inter_pos_estimated = pos_estimated;
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
    output.final_time_move = time_move;
    output.final_time_measure = time_measure;
    output.final_time= time_move + time_measure;
    output.final_precision = error_norm;
    output.final_pos_estimated = pos_estimated;
    output.pos_real = pos_true_node;
    
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
