function dz = dynamics(z,u,param)
% dz = dynamics(z,u,param)
%
% Computes the first-order form of the dynamics for the combined chain
% integrator and pendulum system
%
% INPUTS:
%   z = [x;v1;v2];
%   u = [u1;u2];
%
% OUTPUTS:
%   dz = dz/dt
%

x = z(1,:);
v1 = z(2,:);
% v2 = z(3,:);   %Unused
u1 = u(1,:);
u2 = u(2,:);

% Pendulum physics
dv1 = pendulum(x,v1,u1,param);

% Integrator chain physics:
dx = v1;
dv2 = u2;

% Combine:
dz = [dx;dv1;dv2];

end