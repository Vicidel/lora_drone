% does circles around estimate and continuous measures
function output = sim2_1drone_continuous()

    clear all; close all;
    param;

    % init
    global time_move;
    global time_measure;    % will be false as we continuously measure signal, no more 5 second of hover at each time
    time_move = 0; 
    time_measure = 0;
    angle_count = 0;
    algo_loop = 1;
    time_last_measure = 0;
    dataset = [];
    
    % create first figure
    if plot_bool
        figure();
        plot_tri(pos_true_node, 'ro'); grid on; hold on;
        plot_tri(pos_network_estimate, 'co');
    end

    % drone takes off
    pos_drone = pos_drone + [0, 0, altitude];
    time_move = time_move + 10;
    
    % drones goes into start position
    pos_drone = pos_drone + [pattern_radius, 0, 0];
    time_move = time_move + pattern_radius/drone_speed;
    

    
    % localization
    while time_move+0 < time_limit
        
        % fly drone in a circle
        pos_drone = pattern_center + [pattern_radius*cos(pattern_angle_start + angle_count*pattern_anglerad_per_second), pattern_radius*sin(pattern_angle_start + angle_count*pattern_anglerad_per_second), 0];
        angle_count = angle_count + 1;  % go to next position
        time_move = time_move + 1;      % angle_count is angle done in one second
        
        % five seconds since last measure
        if time_move - time_last_measure > 5
            
            % make measure
            time_last_measure = time_move;
            dataset = [dataset; pos_drone, get_noisy_signal(pos_true_node, pos_drone, signal_type, 10)];
            
            % plot
            if plot_bool
                plot_tri(pos_drone, 'bo');
            end
        end
        
        % done on full circle
        if angle_count*pattern_anglerad_per_second > 2*pi
            
            % make estimation
            [x, y] = get_position_dataset(dataset, signal_type);
            pos_estimated = [x, y, 0];
            
            % plot
            if plot_bool
                for i=2:6:size(dataset)
                    plot_circle(dataset(i,1), dataset(i,2), func_signal_to_distance(dataset(i,4), signal_type));
                end
                plot_tri(pos_estimated, 'kx');
                xlabel('x position [m]')
                ylabel('y position [m]')
                zlabel('z position [m]')
                title('Node localization algorithm');
                legend('Node position', 'Last estimate', 'Measuring positions');
                view(0, 90); axis equal;
            end
            
            % print in terminal
            if print_bool
                fprintf('Error after %d loop(s): dx=%.2f, dy=%.2f, norm=%.2f\n', algo_loop, abs(pos_estimated(1) - pos_true_node(1)), abs(pos_estimated(2) - pos_true_node(2)), norm([abs(pos_estimated(1) - pos_true_node(1)), abs(pos_estimated(2) - pos_true_node(2))])); 
                fprintf('Time passed: %.2f\n', time_move+0);
            end 
            
            % create second circle around estimate
            pattern_center = pos_estimated;
            pattern_radius = pattern_radius_v2;
            pattern_anglerad_per_second = pattern_anglerad_per_second_v2;  
            
            % find closest starting point
            pattern_angle_start = pi;    % !!!!! to change !!!!!
            
            % break condition
            if algo_loop == algo_loops_todo
                break;
            end
            
            % move to new starting point
            pos_drone_old = pos_drone;
            pos_drone = pos_estimated + [pattern_radius*cos(pattern_angle_start), pattern_radius*sin(pattern_angle_start), 0];
            time_move = time_move + norm(pos_drone-pos_drone_old);
            
            % reset angle_count and dataset                   
            if print_bool fprintf('Restarting algorithm around found position\n\n'); end
            if plot_bool
                figure(); 
                plot_tri(pos_true_node, 'ro'); grid on; hold on;
                plot_tri(pos_estimated, 'co');
            end
            angle_count = 0;
            dataset = [];
            algo_loop = algo_loop + 1;
        end
        
        % slows down
        pause(0.01);
    end
    
    % time limit reached
    if time_move + 0 >= time_limit-2 && print_bool
        fprintf('\nTime limit reached (%.2f minutes passed)\n', (time_move)/60);
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

% plots a circle in x, y, radius r 
function plot_circle(x, y, r)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    plot(xunit, yunit);
end

% make estimation base on dataset samples
function [xout, yout] = get_position_dataset(dataset, signal_type)
    
    [nb_points,~] = size(dataset);
    sol_x = zeros(nb_points,1);
    sol_y = zeros(nb_points,1);
    
    for i=1:nb_points
        % get index of three random points
        index1 = 1+floor(rand()*nb_points);
        index2 = 1+floor(rand()*nb_points);
        index3 = 1+floor(rand()*nb_points);
        
        % do trilateration
        [sol_x(i), sol_y(i)] = get_position_tri(dataset(index1,1), dataset(index1,1), func_signal_to_distance(dataset(index1,4), signal_type), ...
                                                dataset(index2,1), dataset(index2,2), func_signal_to_distance(dataset(index2,4), signal_type), ...
                                                dataset(index3,1), dataset(index3,2), func_signal_to_distance(dataset(index3,4), signal_type));
    end
    
    % make median value ignoring all NaN
    xout = nanmedian(sol_x);
    yout = nanmedian(sol_y);
end