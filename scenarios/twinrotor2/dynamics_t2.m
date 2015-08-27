%% dynamics_t1.m
% *Summary:* Implements ths ODE for simulating the twinrotor1 dynamics.
%  It is a simple bar with a centered fulcrum and forces on each end.
%
%    function dz = dynamics_cp(t, z, f1, f2)
%
% *Input arguments:*
%
%	t     current time step (called from ODE solver)
%   z     state                                                    [2 x 1]
%   f1    force 1
%   f2    force 2
%
% *Output arguments:*
%
%   dz    if 4 input arguments:     state derivative wrt time
%         else:                     total mechanical energy
%
% Note: It is assumed that the state variables are of the following order:
%       theta:    [rad]   angle
%       dtheta:   [rad/s] angular velocity
%       dx:       [m/s]   horizontal velocity
%       dz:       [m/s]   vertical velocity
%
% Copyright (C) 2008-2013 by
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
% Modified by Mark Paluta

function dz = dynamics_t2(t,z,f1,f2)
%% Code

I = 1;    % [kg*m^2]  moment of inertia of system
m = 1;    % [kg]      mass of system
b = 0.01; % [s*N/rad] friction coefficient

% if nargin==4
  dz = zeros(4,1);
  dz(1) = z(2);
  dz(2) = (f2(t)-f1(t)-b*z(2))/I;
  dz(3) = (-f2(t)-f1(t))*sin(z(1))/m;
  dz(4) = (f2(t)+f1(t))*cos(z(1))/m;
% else
%   dz = (M+m)*z(2)^2/2 + 1/6*m*l^2*z(3)^2 + m*l*(z(2)*z(3)-g)*cos(z(4))/2;
% end