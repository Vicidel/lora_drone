% follows a gradient of signal strength to find maximum, flies at 10m height
function output = sim2_1drone_continuous()

    clear all; close all;
    param;

    % init
    global time_move;
    global time_measure;
    time_move = 0; 
    time_measure = 0;
    angle_count = 0;
    time_last_measure = 0;
    dataset = [pos_drone, func_distance_to_signal(300, signal_type)];
    
    % create first figure
    if plot_bool
        figure();
        plot_tri(pos_true_node, 'ro'); grid on; hold on;
        plot_tri(pos_network_estimate, 'co');
        plot_tri(pos_drone, 'bo');
    end

    % drone takes off
    pos_drone = pos_drone + [0, 0, altitude];
    time_move = time_move + 10;
    
    % drones goes into start position
    pos_drone = pos_drone + [pattern_radius, 0, 0];
    time_move = time_move + pattern_radius/drone_speed;
    

    
    % localization
    while time_move+0 < time_limit    % we don't consider measuring time as incorrect
        
        % fly drone in a circle
        pos_drone = pattern_center + [pattern_radius*cos(pattern_angle_start + angle_count*pattern_anglerad_per_second), pattern_radius*sin(pattern_angle_start + angle_count*pattern_anglerad_per_second), 0];
        angle_count = angle_count + 1;  % go to next position
        time_move = time_move + 1;      % angle_count is angle done in one second
        
        % five seconds since last measure
        if time_move - time_last_measure > 5
            
            % make measure
            time_last_measure = time_move;
            dataset = [dataset; pos_drone, get_noisy_signal(pos_true_node, pos_drone, signal_type, 1)];
            
            % plot
            if plot_bool
                plot_tri(pos_drone, 'bo');
            end
        end
        
        % can begin making estimation
        if angle_count > 4
            pos_estimated = pos_true_node; % !!!!! to change !!!!!
        end
        
        % done on full circle
        if angle_count*pattern_anglerad_per_second > 2*pi
            
            % recenter on estimation
            pattern_center = pos_estimated;
            
            % create second circle around estimate
            pattern_radius = pattern_radius_v2;
            pattern_anglerad_per_second = pattern_anglerad_per_second_v2;  
            
            % find closest starting point
            pattern_angle_start = 0;    % !!!!! to change !!!!!
            
            % move to new starting point
            pos_drone_old = pos_drone;
            pos_drone = pos_estimated + [pattern_radius*cos(pattern_angle_start), pattern_radius*sin(pattern_angle_start), 0];
            time_move = time_move + norm(pos_drone-pos_drone_old);
            
            % reset angle_count
            angle_count = 0;
        end
        
        time_move
        
        % slows down
        pause(0.01);
    end
    
    % time limit reached
    if time_move + 0 >= time_limit-2 && print_bool
        fprintf('\nTime limit reached (%.2f minutes passed)\n', (time_move)/60);
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
        fprintf('Found in t=%.1f seconds (%.1f moving and %.1f measuring)\n', time_move, time_move, 0);
    end
    
    % create output
    output.time_move_final = time_move;
    output.time_measure_final = 0;
    output.time_final = time_move + 0;
    output.final_precision = error_norm;
    output.pos_real = pos_true_node;
    output.pos_estimated = pos_estimated;
    
end



% plot a vector of 3x1 in defined color
function plot_tri(vector, color)
    plot3(vector(1), vector(2), vector(3), color);
end