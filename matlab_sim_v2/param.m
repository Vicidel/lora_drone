%% sets the parameters used in the simulations

% general parameters
time_limit = 30*60;     % 15 minutes
drone_speed = 1;        % 1 m/s
signal_type = 'esp';    % can be 'esp' or 'rssi'
altitude = 10;          % flies at 10m
number_measures = 2;    % makes 2 measures at each point

% boolean for plotting and printing
plot_bool = false;       
plot_movement_bool = false;
print_bool = false;

% node position and estimate from network
pos_true_node = [0, 0, 0];
pos_network_error = 150; 
pos_network_estimate = pos_true_node + [rand()*pos_network_error*cos(rand()*2*pi), rand()*pos_network_error*sin(rand()*2*pi), 0];

% name of the running script
file_run = dbstack(1);
file_run_name = file_run.name;

%% for gradient follow
switch file_run_name 
    
    case 'sim2_1drone_gradient'
        % distance increment
        dist_increment = 10;
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
        
        
end