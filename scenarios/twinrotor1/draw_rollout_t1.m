%% draw_rollout_t1.m
% *Summary:* Script to draw the most recent trajectory of the twinrotor1
% system together with the predicted uncertainties around the tip of the 
% pendulum
%
% Copyright (C) 2008-2013 by 
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
% Modified by Mark Paluta
%
%% High-Level Steps
% # For each time step, plot the observed trajectory and the predicted
% means and covariances of the Cartesian coordinates of the tip of the
% pendulum

%% Code

% Loop over states in trajectory (= time steps)
for r = 1:size(xx,1)
%    disp('*******************');
  if exist('j','var') && ~isempty(M{j})
    draw_t1(latent{j}(r,1), latent{j}(r,end-1), latent{j}(r,end), cost,  ...
      ['trial # ' num2str(j+J) ', T=' num2str(H*dt) ' sec'], ...
      ['total experience (after this trial): ' num2str(dt*size(x,1)) ...
      ' sec'], M{j}(:,r), Sigma{j}(:,:,r));
  else
     draw_t1(latent{jj}(r,1), latent{jj}(r,end-1),latent{jj}(r,end), cost,  ...
      ['(random) trial # ' num2str(jj) ', T=' num2str(H*dt) ' sec'], ...
      ['total experience (after this trial): ' num2str(dt*size(x,1)) ...
      ' sec'])
  end
  pause(dt);
end
