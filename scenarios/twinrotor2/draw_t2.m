%% draw_t1.m
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

function draw_t2(theta, dtheta, dx, dz, force1, force2, cost, text1, text2, M, S)
%% Code

l = 1;  
maxU = 1;
xmin = -1; 
xmax = 1;
velocity_scaling = 1;

% Compute rotor coordinates
x_pend = l*cos(theta);
y_pend = l*sin(theta);
pendulum = [-x_pend, x_pend;-y_pend, y_pend];

clf; hold on
plot(-l,0,'k+','MarkerSize',20,'linewidth',2) % Target point
plot(l,0,'k+','MarkerSize',20,'linewidth',2)

% Plot force
plot([0 force1/maxU*xmax],[-0.3, -0.3],'g','linewidth',10)
plot([0 force2/maxU*xmax],[-0.5, -0.5],'g','linewidth',10)

% Plot reward
reward = 1-cost.fcn(cost,[theta, dtheta, dx, dz]', zeros(4));
plot([0 reward*xmax],[-0.7, -0.7],'y','linewidth',10)

% Plot the rotor
plot(pendulum(1,:),pendulum(2,:), 'b','linewidth',4)

% Plot the fulcrum
plot(0,0,'y.','markersize',24)

% Plot velocities
plot([0, dx*velocity_scaling], [0, 0],'r','linewidth',3) % x velocity
plot([0, 0], [0, dz*velocity_scaling],'r','linewidth',3) % z velocity

% plot ellipse around tip of pendulum (if M, S exist)
try
  [M1 S1] = getPlotDistr_t1(M,S,2*l);
  error_ellipse(S1,M1,'style','b');
catch
end

% Text
text(0,-0.3,'applied force 1')
text(0,-0.5,'applied force 2')
text(0,-0.7,'immediate reward')
if exist('text1','var')
  text(0,-0.9, text1)
end
if exist('text2','var')
  text(0,-1.1, text2)
end

set(gca,'DataAspectRatio',[1 1 1],'XLim',[xmin xmax],'YLim',[-1.4 1.4]);
axis off;
drawnow;