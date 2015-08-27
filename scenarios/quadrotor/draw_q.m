%% draw_q.m
% *Summary:* Draw the cart-pole system with reward, applied force, and 
% predictive uncertainty of the tip of the pendulum
%
%    function draw_cp(x, theta, force, cost, text1, text2, M, S)
%
%
% *Input arguments:*
%
%		x          position of the cart
%   theta      angle of pendulum
%   force      force applied to cart
%   cost       cost structure
%     .fcn     function handle (it is assumed to use saturating cost)
%     .<>      other fields that are passed to cost
%   M          (optional) mean of state
%   S          (optional) covariance of state
%   text1      (optional) text field 1
%   text2      (optional) text field 2
%
%
% Copyright (C) 2008-2013 by
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
% Last modified: 2013-03-07

function draw_q(phi, theta, p, q, r, u, v, w, ...
    force1, force2, force3, force4, cost, text1, text2, M, S)
%% Code

% For plotting purposes, x and y are in first intermediate frame of
% inertial to body rotation (yaw rotation accounted for).

l = .06;  
maxU = .1;
xmin = -.1; 
xmax = .1;
velocity_scaling = .01;

% Compute rotor coordinates
offset_angle = 45  *pi/180;
c = l*cos(offset_angle);
s = l*sin(offset_angle);

r1 = [c; s; 0]; % location of propellers (body frame)
r2 = [-s; c; 0];
r3 = [-c; -s; 0];
r4 = [s; -c; 0];

cr = cos(phi); % cosine of roll
sr = sin(phi); % sine
cp = cos(theta); % cosine of pitch
sp = sin(theta); % sine
Euler1 = [cp 0 sp ; 0 1 0 ; -sp 0 cp]; % Euler matrix #2 (pitch)
Euler2 = [1 0 0 ; 0 cr sr ; 0 -sr cr]; % Euler matrix #1 (roll)

R1 = Euler2*Euler1*r1; % DCM     [roll]*[pitch]    frame already yawed
R2 = Euler2*Euler1*r2;
R3 = Euler2*Euler1*r3;
R4 = Euler2*Euler1*r4;

% set up window
clf; hold on
view(30,30) % (azimuth, elevation)

% For actual plotting, we now swap x and y and make z = -z (Matlab's
% convention  is x right, y forward, z up).

% Plot x and y axes to define yawed x and y frame
plot3([0 0],[0 2*l],[0 0],'k','linewidth',2); % yawed x axis
plot3([0 2*l],[0 0],[0 0],'k','linewidth',2); % yawed y axis
plot3([0 0],[0 0],-[0 2*l],'k','linewidth',2); % z axis
plot3(0,2.5*l,0,'rx','MarkerSize', 10,'linewidth',2); % forward direction

% Plot target points
plot3(r1(2),r1(1),0,'k+','MarkerSize',20,'linewidth',2)
plot3(r2(2),r2(1),0,'k+','MarkerSize',20,'linewidth',2)
plot3(r3(2),r3(1),0,'k+','MarkerSize',20,'linewidth',2)
plot3(r4(2),r4(1),0,'k+','MarkerSize',20,'linewidth',2)

% Plot the rotor
plot3([R1(2) R3(2)],[R1(1) R3(1)],-[R1(3) R3(3)], 'b','linewidth',4)
plot3([R2(2) R4(2)],[R2(1) R4(1)],-[R2(3) R4(3)], 'b','linewidth',4)

% Plot the fulcrum
plot3(0,0,0,'y.','markersize',24)

% Title
title({text1, text2}')

set(gca,'DataAspectRatio',[1 1 1],'XLim',[xmin xmax],'YLim',[-1.4/10 1.4/10],'ZLim',[-1.4/10 1.4/10]);
axis off;
drawnow;