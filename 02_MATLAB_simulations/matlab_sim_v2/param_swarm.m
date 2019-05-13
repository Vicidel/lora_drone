% parameters for an oflati saber swarm
% taken from lis-epfl/olfati-saber-swarm repository
% all credits to Enrica Soria and the LIS-EPFL lab


% Simulated time
P.dt = 0.1;     % time step
P.T  = 300;     % time of simulation [s]

% Number of agents & aimed inter-agent distance
P.N         = 3;
P.r_boid    = 0.2; % radius of a spheric boid
P.d         = 5;

% Initial positions and velocities
P.X0 	= [-5,-5,0]';
P.X     = 6;
P.V0 	= [0,0,0]';
P.V 	= 0;
P.seed  = 5;

% Velocity of migration
P.v_migration = [0 1 0]';        

% Max radius of influence - Metric distance
P.r   = 150;

% Max number of neighbors - Topological distance
P.max_neig = 5;

% Velocity and acceleration bounds
P.max_a     = 4;
P.max_v     = 3;
