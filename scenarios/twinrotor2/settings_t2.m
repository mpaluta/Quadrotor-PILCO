%% settings_t2.m
% *Summary:* Script set up the twinrotor2 scenario
%
% Copyright (C) 2008-2013 by 
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
% Modified by Mark Paluta
%
%% High-Level Steps
% # Define state and important indices
% # Set up scenario
% # Set up the plant structure
% # Set up the policy structure
% # Set up the cost structure
% # Set up the GP dynamics model structure
% # Parameters for policy optimization
% # Plotting verbosity
% # Some array initializations

%% Code

rand('seed',1); randn('seed',1); format short; format compact; 
% include some paths
try
  rd = '../../';
  addpath([rd 'base'],[rd 'util'],[rd 'gp'],[rd 'control'],[rd 'loss']);
catch
end


% 1. Define state and important indices

% 1a. Full state representation (including all augmentations)
%
%  1  theta      angle of the twinrotor
%  2  dtheta     angular velocity
%  3  dx         horizontal velocity (East positive)
%  4  dz         vertical velocity (North positive)
%  5  sin(theta) complex representation ...
%  6  cos(theta) of theta
%  7  f1         force 2
%  8  f2         force 1

% 1b. Important indices
% odei  indicies for the ode solver
% augi  indicies for variables augmented to the ode variables
% dyno  indicies for the output from the dynamics model and indicies to loss
% angi  indicies for variables treated as angles (using sin/cos representation)
% dyni  indicies for inputs to the dynamics model
% poli  indicies for the inputs to the policy
% difi  indicies for training targets that are differences (rather than values)

odei = [1 2 3 4];            % varibles for the ode solver
augi = [];                   % variables to be augmented
dyno = [1 2 3 4];            % variables to be predicted (and known to loss)
angi = [1];                  % angle variables
dyni = [2 3 4 5 6];          % variables that serve as inputs to the dynamics GP
poli = [2 3 4 5 6];          % variables that serve as inputs to the policy
difi = [1 2 3 4];            % variables that are learned via differences


% 2. Set up the scenario
dt = 0.10;                         % [s] sampling time
T = 4.0;                           % [s] initial prediction horizon time
H = ceil(T/dt);                    % prediction steps (optimization horizon)
mu_max = [1 1 1 1]';               % max of state values, (bounds for random state generation)
mu0 = randomize_mu(mu_max);        % initial state mean
S0 = diag([0.1 0.1 0.1 0.1].^2);   % initial state covariance
N = 15;                            % number controller optimizations
J = 1;                             % initial J trajectories of length H
K = 1;                             % no. of initial states for which we optimize
nc = 10;                           % number of controller basis functions

% 3. Plant structure
plant.dynamics = @dynamics_t2;                    % dynamics ode function
plant.noise = diag(ones(1,4)*0.01.^2);            % measurement noise
plant.dt = dt;
plant.ctrl = @zoh;                                % controler is zero order hold
plant.odei = odei;
plant.augi = augi;
plant.angi = angi;
plant.poli = poli;
plant.dyno = dyno;
plant.dyni = dyni;
plant.difi = difi;
plant.prop = @propagated;

% 4. Policy structure
policy.fcn = @(policy,m,s)conCat(@congp,@gSat,policy,m,s);% controller 
                                                          % representation
policy.maxU = [1 1];                                      % max. amplitude of 
                                                          % control
[mm ss cc] = gTrig(mu0, S0, plant.angi);                  % represent angles 
mm = [mu0; mm]; cc = S0*cc; ss = [S0 cc; cc' ss];         % in complex plane          
policy.p.inputs = gaussian(mm(poli), ss(poli,poli), nc)'; % init. location of 
                                                          % basis functions
policy.p.targets = 0.1*randn(nc, length(policy.maxU));    % init. policy targets 
                                                          % (close to zero)
policy.p.hyp = repmat(log([1 1 1 0.7 0.7 1 0.01])',1,2);      % initialize policy
                                                          % hyper-parameters

% 5. Set up the cost structure
cost.fcn = @loss_t2;                       % cost function
cost.gamma = 1;                            % discount factor
cost.p = 1;                                % length of pendulum
cost.width = 0.05;                         % cost function width
cost.expl =  0.0;                          % exploration parameter (UCB)
cost.angle = plant.angi;                   % index of angle (for cost function)
cost.target = [0 0 0 0]';                      % target state

% 6. Dynamics model structure
dynmodel.fcn = @gp1d;                % function for GP predictions
dynmodel.train = @train;             % function to train dynamics model
dynmodel.induce = zeros(300,0,1);    % shared inducing inputs (sparse GP)
trainOpt = [300 500];                % defines the max. number of line searches
                                     % when training the GP dynamics models
                                     % trainOpt(1): full GP,
                                     % trainOpt(2): sparse GP (FITC)

% 7. Parameters for policy optimization
opt.length = 150;                        % max. number of line searches
opt.MFEPLS = 30;                         % max. number of function evaluations
                                         % per line search
opt.verbosity = 1;                       % verbosity: specifies how much 
                                         % information is displayed during
                                         % policy learning. Options: 0-3

% 8. Plotting verbosity
plotting.verbosity = 1;            % 0: no plots
                                   % 1: some plots
                                   % 2: all plots

% 9. Some initializations
x = []; y = [];
fantasy.mean = cell(1,N); fantasy.std = cell(1,N);
realCost = cell(1,N); M = cell(N,1); Sigma = cell(N,1);