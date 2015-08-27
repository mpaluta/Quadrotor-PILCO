%% dynamics_q.m
% *Summary:* Implements the ODE for simulating the quadrotor dynamics
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
%   dz:     state derivative wrt time
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

function dz = dynamics_q(t,z,f1,f2,f3,f4)
%% Parameters

% [kg*m^2] Moments of inertia estimated from 45 degree offset angle
Ixx = 9.39e-5;
Iyy = Ixx;
Izz = 1.87e-4;
%Ixy = 0;
Ixz = 0;
%Iyz = 0;
m = .065;    % [kg]      mass of system
l = .06;     % [m]       distance from CG to props
g = 9.8; 
b = 0; % drag term

%% Construct vectors

offset_angle = 45  *pi/180; % measured from positive x-axis clockwise to prop 1
c = l*cos(offset_angle);
s = l*sin(offset_angle);

r1 = [c; s; 0]; % location of propellers (body frame)
r2 = [-s; c; 0];
r3 = [-c; -s; 0];
r4 = [s; -c; 0];

F1 = [0; 0; -m*g/4-f1(t)]; % forces from propellers (body frame)
F2 = [0; 0; -m*g/4-f2(t)];
F3 = [0; 0; -m*g/4-f3(t)];
F4 = [0; 0; -m*g/4-f4(t)];
F = F1 + F2 + F3 + F4;% - .05 * [z(6) z(7) 20*z(8)]';

T1 = cross(r1, F1); % torques from thrusts (body frame)
T2 = cross(r2, F2);
T3 = cross(r3, F3);
T4 = cross(r4, F4);
propTorque = .00117/.1 * (F1+F3-F2-F4); % from spinning prop

Torque = T1+T2+T3+T4 + propTorque;

%% Convert notation
phi = z(1); % roll
theta = z(2); % pitch
p = z(3);
q = z(4);
r = z(5);
u = z(6);
v = z(7);
w = z(8);

Fx = F(1);
Fy = F(2);
Fz = F(3);

L = Torque(1);
M = Torque(2);
N = Torque(3);

%% Define ODEs

dz = zeros(8,1);

dz(1) = p+sin(phi)*tan(theta)*q+cos(phi)*tan(theta)*r;
dz(2) = cos(phi)*q-sin(phi)*r;
dz(3) = (Izz*L+Ixz*N-(Ixz*(Iyy-Ixx-Izz)*p+(Ixz^2+Izz*(Izz-Iyy))*r)*q)/(Ixx*Izz-Ixz^2);
dz(4) = 1/Iyy*(M-(Ixx-Izz)*p*r-Ixz*(p^2-r^2));
dz(5) = (Ixz*L+Ixx*N-(Ixz*(Iyy-Ixx-Izz)*r+(Ixz^2+Ixx*(Ixx-Iyy))*p)*q)/(Ixx*Izz-Ixz^2);
dz(6) = Fx/m-g*sin(theta)+r*v-q*w - b*z(6);
dz(7) = Fy/m+g*sin(phi)*cos(theta)-r*u+p*w - b*z(7);
dz(8) = Fz/m+g*cos(phi)*cos(theta)+q*u-p*v - b*z(8);