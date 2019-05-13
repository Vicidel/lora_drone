%% sets the parameters used in the simulations

% general parameters
time_limit = 15*60;     % 15 minutes
drone_speed = 5;        % m/s
drone_speed_v2 = 2;     % m/s
signal_type = 'esp';    % can be 'esp' or 'rssi'
altitude = 50;          % flies at 10m
number_measures = 5;    % makes 2 measures at each point

% boolean for plotting and printing
plot_bool = true;       
plot_movement_bool = false;
print_bool = true;

% node position and estimate from network
pos_true_node = [0, 0, 0];
pos_network_error = 200; 
pos_network_estimate = pos_true_node + [rand()*pos_network_error*cos(rand()*2*pi), rand()*pos_network_error*sin(rand()*2*pi), 0];

% name of the running script
file_run = dbstack(1);
file_run_name = file_run.name;

%% for gradient follow
switch file_run_name 
    
    case 'sim2_1drone_gradient'
        % distance increment
        dist_increment = 20;
        increment_0p0 = [0, dist_increment, 0];
        increment_0m0 = [0, -dist_increment, 0];
        increment_p00 = [dist_increment, 0, 0];
        increment_m00 = [-dist_increment, 0, 0];
        
        % drone starting position
        pos_drone = pos_network_estimate;
        
    case 'sim2_1drone_trilateration'
        % measuring positions
        size_around_estimation_v1 = 100;
        size_around_estimation_v2 = 30;
        measure_position1 = pos_network_estimate + [0, -size_around_estimation_v1, 0];   % south
        measure_position2 = pos_network_estimate + [size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 0];   % north east	
        measure_position3 = pos_network_estimate + [-size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 0];   % north east	
        
        % drone starting position
        pos_drone = pos_network_estimate;
        
        % number of loops of trilateration
        algo_loops_todo = 2;
        
    case 'sim2_1drone_trilateration_mod'
        % measuring positions
        size_around_estimation_v1 = 100;
        size_around_estimation_v2 = 30;
        angle_total = 120*pi/180;        % angle instead of 360deg
        measure_position1 = pos_network_estimate + [size_around_estimation_v1, 0, 0];   
        measure_position2 = pos_network_estimate + [size_around_estimation_v1*cos(angle_total/2), size_around_estimation_v1*sin(angle_total/2), 0];   
        measure_position3 = pos_network_estimate + [size_around_estimation_v1*cos(angle_total), size_around_estimation_v1*sin(angle_total), 0];   
        
        % drone starting position
        pos_drone = pos_network_estimate;
        
        % number of loops of trilateration
        algo_loops_todo = 2;
        
    case 'sim2_3drone_trilateration'
        % measureing positions
        size_around_estimation_v1 = 100;
        size_around_estimation_v2 = 30;
        measure_position1 = pos_network_estimate + [0, -size_around_estimation_v1, 0];   % south
        measure_position2 = pos_network_estimate + [size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 0];   % north east	
        measure_position3 = pos_network_estimate + [-size_around_estimation_v1*cos(pi/6), size_around_estimation_v1*sin(pi/6), 0];   % north east	
        
        % drone starting position
        pos_drone1 = pos_network_estimate;
        pos_drone2 = pos_network_estimate;
        pos_drone3 = pos_network_estimate;
        
        % number of loops of trilateration
        algo_loops_todo = 2;
        
    case 'sim2_1drone_continuous'
        % drone starting position
        pos_drone = pos_network_estimate;
        
        % drone speed
        drone_speed = 5;        % m/s
        drone_speed_v2 = 5;     % m/s
        
        % pattern
        pattern_center = pos_network_estimate;
        pattern_radius = 120;
        pattern_radius_v2 = 70;
        pattern_angle_start = 0;
        pattern_anglerad_per_second = drone_speed / pattern_radius;
        pattern_anglerad_per_second_v2 = drone_speed_v2 / pattern_radius_v2;
        
        % number of loops
        algo_loops_todo = 2;
        
    case 'sim2_3drone_continuous'
        % drone starting position
        pos_drone1 = pos_network_estimate;
        pos_drone2 = pos_network_estimate;
        pos_drone3 = pos_network_estimate;
        
        % drone speed
        drone_speed = 5;    % slowed down for more measures
        drone_speed_v2 = 1;    % slowed down for more measures
        
        % pattern
        pattern_shape = 'circle';
        pattern_dist_from_estimate = 80;
        pattern_dist_from_estimate_v2 = 50;
        pattern_center1 = pos_network_estimate + [0, -pattern_dist_from_estimate, 0];   % south
        pattern_center2 = pos_network_estimate + [pattern_dist_from_estimate*cos(pi/6), pattern_dist_from_estimate*sin(pi/6), 0];   % north east	
        pattern_center3 = pos_network_estimate + [-pattern_dist_from_estimate*cos(pi/6), pattern_dist_from_estimate*sin(pi/6), 0];   % north west	
        pattern_radius = 70;
        pattern_radius_v2 = 30;
        pattern_angle_start1 = pi/2;
        pattern_angle_start2 = 4*pi/3;
        pattern_angle_start3 = -pi/3;
        pattern_anglerad_per_second = drone_speed / pattern_radius;
        pattern_anglerad_per_second_v2 = drone_speed_v2 / pattern_radius_v2;
        
    case 'sim2_3drone_swarm'
        
        % drone speed
        drone_speed = 5;    % slowed down for more measures
        drone_speed_v2 = 2;    % slowed down for more measures
        
        % swarm parameters
        swarm_spacing = 30;
        swarm_spacing_v2 = 25;
        
        % circle parameters
        pattern_radius = 80;
        pattern_radius_v2 = 60;
        pattern_center = pos_network_estimate;
        pattern_anglerad_per_second = drone_speed / pattern_radius;
        pattern_anglerad_per_second_v2 = drone_speed_v2 / pattern_radius_v2;
        
        % drone starting position
        pos_start = pos_network_estimate;
        pos_swarm_center = pos_start + [pattern_radius, 0, 0];
        pos_drone1 = pos_swarm_center + swarm_spacing*[0, -1, 0];
        pos_drone2 = pos_swarm_center + swarm_spacing*[-sqrt(3)/2, 1/2, 0];
        pos_drone3 = pos_swarm_center + swarm_spacing*[sqrt(3)/2, 1/2, 0];
         
    case 'sim2_3drone_swarm_v2'
        
        % swarm parameters
        swarm_spacing = 5;
        
        % drone starting position
        phi = rand()*2*pi; pos_start = (rand()*700+300)*[cos(phi), sin(phi), 0];
        pos_swarm_center = pos_start;
        pos_drone1 = pos_swarm_center + swarm_spacing*[0, -1, 0];
        pos_drone2 = pos_swarm_center + swarm_spacing*[-sqrt(3)/2, 1/2, 0];
        pos_drone3 = pos_swarm_center + swarm_spacing*[sqrt(3)/2, 1/2, 0];
        
    otherwise
        % drone starting position
        pos_drone = pos_network_estimate;
        
end


%% change based on experiment counter
if isfile('matlab_sim_v2/temp.mat')     % simulation run from 'multiple_run.m'
    plot_bool = false;
    print_bool = false;
    plot_movement_bool = false;
    load('matlab_sim_v2/temp.mat', 'experiment_counter', 'number_experiments');
    if experiment_counter < number_experiments/4
        altitude = 20;
        size_around_estimation_v1 = 100;
        size_around_estimation_v2 = 30;
    elseif experiment_counter < number_experiments/2
        altitude = 20;
        size_around_estimation_v1 = 150;
        size_around_estimation_v2 = 80;
    elseif experiment_counter < number_experiments*3/4
        altitude = 100;
        size_around_estimation_v1 = 100;
        size_around_estimation_v2 = 30;
    else
        altitude = 100;
        size_around_estimation_v1 = 150;
        size_around_estimation_v2 = 80;
    end
end