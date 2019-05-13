% MAIN  --  minimum snap* trajectory
%
% *snap = second derivative of acceleration
%
% For a simple pendulum:
%
% x = position
% v = velocity
% u = torque
%
% ddx = f(x,dx,u);     <-- dynamics
%
% cost = integral(  ddddx^2  );     <-- cost function
%
% subject to:
%   x(0) = 0;
%   x(1) = pi;
%   dx(0) = 0;
%   dx(1) = pi;
%
% How to pose as a standard trajectory optimization problem?
%
% dx = v1;
% dv1 = f(x,v1,u1)
%
% v2 == v1;   % <-- Key line.
% dv2 = a2;
% da2 = j2;  % jerk = derivative of acceleration
% dj2 = u2;  % snap = derivative of jerk
% cost = integral(  u2^2  );
%
%
% NOTES:
%
%   z = [x;v1;v2;a2;j2];
%   u = [u1;u2];
%
% PITFALLS:
%   One major problem with chain integrators is that they need to be
%   carefully scaled. For example, if you make the duration of the
%   trajectory shorter (eg. tF->2) then the trajectory is scaled.
%   Unfortunately, the derivative scales with duration, the acceleration
%   scales with duration squared, jerk scales with the cube of duration,
%   and snap scales with the 4th of duration. This causes problems in the
%   constraint solver inside of FMINCON. 
%
%


clc; clear;
addpath ../../..

%%%% Specify boundary conditions
t0 = 0;
tF = 5;  

maxTorque = 1.0;

param.k = 1.0;  % gravity torque constant for pendulum model
param.b = 0.1;  % viscous damping constant

z0 = [0;0];
zF = [pi;0];



%%%% Pack up boundary conditions
problem.bounds.initialTime.low = t0;
problem.bounds.initialTime.upp = t0;

problem.bounds.finalTime.low = tF;
problem.bounds.finalTime.upp = tF;

problem.bounds.initialState.low = [z0; -inf(3,1)];
problem.bounds.initialState.upp = [z0; inf(3,1)];

problem.bounds.finalState.low = [zF; -inf(3,1)];
problem.bounds.finalState.upp = [zF; inf(3,1)];

problem.bounds.control.low = [-maxTorque; -inf];
problem.bounds.control.upp = [maxTorque; inf];


%%%% Initialize trajectory with a straight line
problem.guess.time = [t0,tF];
problem.guess.state = [[z0, zF]; zeros(3,2)];
problem.guess.control = [zeros(2,1), zeros(2,1)];


%%%% Pack up function handles
problem.func.dynamics = @(t,z,u)(  dynamics(z,u,param)  );
problem.func.pathObj = @(t,z,u)(  pathObjective(u)  );
problem.func.pathCst = @(t,z,u)(  pathConstraint(z)  );


%%%% Choice of solver:
method = 'trapezoid';

switch method
    case 'chebyshev'
        problem.options.method = method;
        problem.options.chebyshev.nColPts = 25;
    case 'trapezoid'
        problem.options.method = method;
        problem.options.trapezoid = 30;
    case 'hermiteSimpson'
        problem.options.method = method;
        problem.options.hermiteSimpson.nSegment = 15;
        problem.options.nlpOpt.MaxFunEvals = 5e4;
    case 'gpops'
        problem.options.method = 'gpops';
    otherwise
        error('invalid method')
end

%%%% Solve
soln = optimTraj(problem);


%%%% Unpack the solution

tGrid = soln.grid.time;
xGrid = soln.grid.state(1, :);
v1Grid = soln.grid.state(2, :);
v2Grid = soln.grid.state(3, :);
a2Grid = soln.grid.state(4, :);
j2Grid = soln.grid.state(5, :);
u1Grid = soln.grid.control(1, :);
dj2Grid = soln.grid.control(2, :);

t = linspace(tGrid(1), tGrid(end), 100);
z = soln.interp.state(t);
u = soln.interp.control(t);
x = z(1,:);
v1 = z(2,:);
v2 = z(3,:);
a2 = z(4,:);
j2 = z(5,:);
u1 = u(1,:);
dj2 = u(2,:);

%%%% Plot the trajectory against time
figure(1); clf;

subplot(3,2,1); hold on;
plot(t,x)
plot(tGrid,xGrid,'ko','MarkerSize',8,'LineWidth',2);
title('position (angle)')

subplot(3,2,3); hold on;
plot(t,v1)
plot(t,v2)
plot(tGrid,v1Grid,'ko','MarkerSize',8,'LineWidth',2);
plot(tGrid,v2Grid,'ko','MarkerSize',8,'LineWidth',2);
title('velocity (angular rate)')
legend('v1','v2')

subplot(3,2,5); hold on;
plot(t([1,end]),[1,1]*maxTorque,'k--','LineWidth',1);
plot(t([1,end]),-[1,1]*maxTorque,'k--','LineWidth',1);
plot(t,u1)
plot(tGrid,u1Grid,'ko','MarkerSize',8,'LineWidth',2);
title('torque')

subplot(3,2,2); hold on;
plot(t,a2)
plot(tGrid,a2Grid,'ko','MarkerSize',8,'LineWidth',2);
title('acceleration')

subplot(3,2,4); hold on;
plot(t,j2)
plot(tGrid,j2Grid,'ko','MarkerSize',8,'LineWidth',2);
title('jerk')

subplot(3,2,6); hold on;
plot(t,dj2)
plot(tGrid,dj2Grid,'ko','MarkerSize',8,'LineWidth',2);
title('snap')





