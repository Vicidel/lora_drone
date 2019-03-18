%% sets the parameters used in the simulations

% general parameters
time_limit = 15*60;     % 15 minutes
drone_speed = 1;        % 1 m/s
signal_type = 'esp';    % can be 'esp' or 'rssi'

% boolean for plotting and printing
plot_bool = true;       
print_bool = true;

% node position and estimate from network
pos_true_node = [0, 0, 0];
pos_network_error = 150; 
pos_network_estimate = pos_true_node + [rand()*pos_network_error*cos(rand()*2*pi), rand()*pos_network_error*sin(rand()*2*pi), 0];

% drone starting position
pos_drone = pos_network_estimate;

% name of the running script
file_run = dbstack(1);

%% for gradient follow
if file_run.name == 'sim2_1drone_gradient'
    
    % distance increment
    dist_increment = 10;
    increment_0p0 = [0, dist_increment, 0];
    increment_0m0 = [0, -dist_increment, 0];
    increment_p00 = [dist_increment, 0, 0];
    increment_m00 = [-dist_increment, 0, 0];
    
    number_measures = 2;        % makes 2 measures at each point
    
    altitude = 10;      % flies at 10m
end