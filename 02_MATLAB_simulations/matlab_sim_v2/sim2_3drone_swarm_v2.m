% three drones in swarming motion collecting data for trilateration
function output = sim2_3drone_swarm_v2()

    clear all; close all;
    param;

    % init
    global time_move;
    global time_measure;
    time_move = 0; 
    time_measure = 0;
    state = 0;
    measure_start_bool = false;
    dataset = [];
    time_last_measure = 0;

    % create first figure
    if plot_bool
        figure();
        plot_tri(pos_true_node, 'mo'); grid on; hold on;
        plot_tri(pos_network_estimate, 'co');
        view(0, 90); axis equal;
    end

    % drone takes off (and moves from each other)
    time_move = time_move + 20 + swarm_spacing;
    pos_drone1 = pos_drone1 + [0, 0, altitude];
    pos_drone2 = pos_drone2 + [0, 0, altitude];
    pos_drone3 = pos_drone3 + [0, 0, altitude];
    pos_swarm_center = [mean([pos_drone1(1), pos_drone2(1), pos_drone3(1)]), ...
                        mean([pos_drone1(2), pos_drone2(2), pos_drone3(2)]), ...
                        mean([pos_drone1(3), pos_drone2(3), pos_drone3(3)])];
    
    % create first swarm heading
    swarm_migration_direction = [0, 0, 0];

    
    % localization
    while time_move+0 < time_limit
        
        % moves drone based on heading and swarming
        [pos_new_drone1, pos_new_drone2, pos_new_drone3] = func_swarm(pos_swarm_center, swarm_migration_direction, swarm_spacing);
        pos_drone1 = pos_new_drone1 + swarm_migration_direction;
        pos_drone2 = pos_new_drone2 + swarm_migration_direction;
        pos_drone3 = pos_new_drone3 + swarm_migration_direction;
        time_move = time_move + norm(swarm_migration_direction)/drone_speed;
        
        % plot
        if plot_bool
            plot_tri(pos_drone1, 'ro');
            plot_tri(pos_drone2, 'go');
            plot_tri(pos_drone3, 'bo');
        end

        % signal becomes good, start measures
        if get_noisy_signal(pos_true_node, pos_drone1, signal_type, 1) > -130
            measure_start_bool = true;
        end
        
        % do measures
        if measure_start_bool

            % five seconds since last measure
            if time_move - time_last_measure > 5

                % make measure for the three drones and add them to the dataset
                time_last_measure = time_move;
                dataset = [dataset; pos_drone1, get_noisy_signal(pos_true_node, pos_drone1, signal_type, 1)];
                dataset = [dataset; pos_drone2, get_noisy_signal(pos_true_node, pos_drone2, signal_type, 1)];
                dataset = [dataset; pos_drone3, get_noisy_signal(pos_true_node, pos_drone3, signal_type, 1)];
                
                % temporary estimate
                [x, y] = get_position_dataset(dataset, signal_type);
                pos_estimated = [x, y, 0];
                if plot_bool plot_tri(pos_estimated, 'mx'); end
            end
        end
        
        % create swarm heading and spacing
        switch state 
            case 0
                % get swarm heading
                pos_swarm_center = [mean([pos_drone1(1), pos_drone2(1), pos_drone3(1)]), ...
                                    mean([pos_drone1(2), pos_drone2(2), pos_drone3(2)]), ...
                                    mean([pos_drone1(3), pos_drone2(3), pos_drone3(3)])];
                delta = pos_network_estimate - pos_swarm_center;
                swarm_migration_direction = delta/norm(delta);
                
                % increase spacing based on measures done
                if measure_start_bool && size(dataset, 1)>1
                    swarm_spacing = 5*size(dataset, 1);
                end
                
                % enough datapoints to have first estimate
                if sum(swarm_migration_direction==zeros(1,3))==3 || size(dataset, 1)>20
                    if print_bool fprintf('Next state, chasing the estimation, time is %.2f\n', time_move); end
                    state = 1;
                    swarm_spacing = 100;
                    dataset = dataset(end-5:end, :);
                end
                
            case 1
                % get swarm heading
                pos_swarm_center = [mean([pos_drone1(1), pos_drone2(1), pos_drone3(1)]), ...
                                    mean([pos_drone1(2), pos_drone2(2), pos_drone3(2)]), ...
                                    mean([pos_drone1(3), pos_drone2(3), pos_drone3(3)])];
                delta = pos_estimated - pos_swarm_center;
                swarm_migration_direction = delta/norm(delta);
                
                % change spacing based on measures done
                if size(dataset, 1)>15
                    swarm_spacing = 100 - 2*(size(dataset, 1)-15);
                end
                
                % change state
                if swarm_spacing < 70       %% TODO: change to threshold reached by filtering
                    state = 2;
                end
                
            case 2
                % drone hover
                swarm_migration_direction = zeros(3,1);
                
                % make more measures
                if print_bool fprintf('Hovering and making more final measures\n'); end
                nb_measures_final = 3;
                dataset = [dataset; pos_drone1, get_noisy_signal(pos_true_node, pos_drone1, signal_type, nb_measures_final)];
                dataset = [dataset; pos_drone2, get_noisy_signal(pos_true_node, pos_drone2, signal_type, nb_measures_final)];
                dataset = [dataset; pos_drone3, get_noisy_signal(pos_true_node, pos_drone3, signal_type, nb_measures_final)];
                time_move = time_move + 5*3*nb_measures_final;
                
                % make multiple estimates
                for i=1:10
                    [x(i), y(i)] = get_position_dataset(dataset, signal_type);
                end
                pos_estimated = [median([x;y],2)', 0];
                
                % plot
                if plot_bool 
                    plot_tri(pos_estimated, 'kx');
                    xlabel('x position [m]')
                    ylabel('y position [m]')
                    zlabel('z position [m]')
                    title('Node localization algorithm');
                    legend('Node position', 'Last estimate', 'Measuring positions for drone 1', 'Measuring positions for drone 2', 'Measuring positions for drone 3');
                    view(0, 90); axis equal;     
                end
                
                % stop program
                break;
        end
        
        % slows down if launched from file
        if ~isfile('matlab_sim_v2/temp.mat') pause(0.005); end
    end
    
    % time limit reached
    if 0+time_move >= time_limit-2
        % print
        if print_bool fprintf('\nTime limit reached (%.2f minutes passed)\n', (time_move+0)/60); end
        
        % make multiple estimates
        for i=1:10
            [x(i), y(i)] = get_position_dataset(dataset, signal_type);
        end
        pos_estimated = [median([x;y],2)', 0];
        
        % plot
        if plot_bool 
            plot_tri(pos_estimated, 'kx');
            xlabel('x position [m]')
            ylabel('y position [m]')
            zlabel('z position [m]')
            title('Node localization algorithm');
            legend('Node position', 'Last estimate', 'Measuring positions for drone 1', 'Measuring positions for drone 2', 'Measuring positions for drone 3');
            view(0, 90); axis equal;     
        end
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
        fprintf('Found in t=%.1f seconds (%.1f moving and %.1f measuring)\n', time_move+0, time_move, 0);
    end
    
    % create output
    output.final_time_move = time_move;
    output.final_time_measure = 0;
    output.final_time= time_move + 0;
    output.final_precision = error_norm;
    output.final_pos_estimated = pos_estimated;
    output.pos_real = pos_true_node;
    
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
    sol_x = nan(nb_points,1);
    sol_y = nan(nb_points,1);
    
    for i=1:1:nb_points
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

% swarm movement, for now dummy versions, always fly in triangular pattern
function [pos_new_drone1, pos_new_drone2, pos_new_drone3] = func_swarm(pos_swarm_center, swarm_migration_direction, swarm_spacing)

    pos_new_center = pos_swarm_center + swarm_migration_direction;
    
    pos_new_drone1 = pos_new_center + swarm_spacing*[0, -1, 0];
    pos_new_drone2 = pos_new_center + swarm_spacing*[-sqrt(3)/2, 1/2, 0];
    pos_new_drone3 = pos_new_center + swarm_spacing*[sqrt(3)/2, 1/2, 0];
    
end