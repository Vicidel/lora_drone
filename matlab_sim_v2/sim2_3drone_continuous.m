% three drones in continuous movement collecting data for trilateration
function output = sim2_3drone_continuous()

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

    % create first figure
    if plot_movement_bool  && plot_bool
        figure(1);
    end

    % drone takes off
    time_move = time_move + 20;
    pos_drone1 = pos_drone1 + [0, 0, altitude];
    pos_drone2 = pos_drone2 + [0, 0, altitude];
    pos_drone3 = pos_drone3 + [0, 0, altitude];

    
    % localization
    while time_move+time_measure < time_limit
    
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
