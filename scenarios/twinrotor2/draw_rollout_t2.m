%% draw_rollout_t2.m
% *Summary:* Script to draw the most recent trajectory of the twinrotor2
% system
%
% Copyright (C) 2008-2013 by 
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
% Modified by Mark Paluta
%
%% High-Level Steps
% # For each time step, plot the observed trajectory

%% Code

% Loop over states in trajectory (= time steps)
for r = 1:size(xx,1)
  if exist('j','var') && ~isempty(M{j})
    draw_t2(latent{j}(r,1), latent{j}(r,2),latent{j}(r,3),latent{j}(r,4), ...
        latent{j}(r,end-1), latent{j}(r,end), cost,  ...
      ['trial # ' num2str(j+J) ', T=' num2str(H*dt) ' sec'], ...
      ['total experience (after this trial): ' num2str(dt*size(x,1)) ...
      ' sec'], M{j}(:,r), Sigma{j}(:,:,r));
  else
     draw_t2(latent{jj}(r,1), latent{jj}(r,2),latent{jj}(r,3),latent{jj}(r,4), ...
         latent{jj}(r,end-1),latent{jj}(r,end), cost,  ...
      ['(random) trial # ' num2str(jj) ', T=' num2str(H*dt) ' sec'], ...
      ['total experience (after this trial): ' num2str(dt*size(x,1)) ...
      ' sec'])
  end
  pause(dt);
end
