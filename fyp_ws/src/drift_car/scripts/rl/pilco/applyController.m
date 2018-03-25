%% applyController.m
% *Summary:* Script to apply the learned controller to a (simulated) system
%
% Copyright (C) 2008-2013 by
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
% Last modified: 2013-06-04
%
%% High-Level Steps
% # Generate a single trajectory rollout by applying the controller
% # Generate many rollouts for testing the performance of the controller
% # Save the data

%% Code

% 1. Generate trajectory rollout given the current policy
if isfield(plant,'constraint'), HH = maxH; else HH = H; end
[xx, yy, realCost{j+J}, latent{j}] = ...
  rollout(gaussian(mu0, S0), policy, HH, plant, cost);
disp(xx);                           % display states of observed trajectory
x = [x; xx]; y = [y; yy];                            % augment training set
if plotting.verbosity > 0
  if ~ishandle(3); figure(3); else set(0,'CurrentFigure',3); end
  hold on; plot(1:length(realCost{J+j}),realCost{J+j},'r'); 
  xlabel('Time Steps');
  ylabel('Cost');
  drawnow;
end

% 2. Make many rollouts to test the controller quality
if plotting.verbosity > 1
  lat = cell(1,10);
  for i=1:10
    [~,~,~,lat{i}] = rollout(gaussian(mu0, S0), policy, HH, plant, cost);
  end
  
  if ~ishandle(4); figure(4); else set(0,'CurrentFigure',4); end; clf(4);
  
  ldyno = length(dyno);
  for i=1:ldyno       % plot the rollouts on top of predicted error bars
    subplot(ceil(ldyno/sqrt(ldyno)),ceil(sqrt(ldyno)),i); hold on;
    errorbar( 0:length(M{j}(i,:))-1, M{j}(i,:), ...
      2*sqrt(squeeze(Sigma{j}(i,i,:))) );
    for ii=1:10
      plot( 0:size(lat{ii}(:,dyno(i)),1)-1, lat{ii}(:,dyno(i)), 'r' );
    end
    plot( 0:size(latent{j}(:,dyno(i)),1)-1, latent{j}(:,dyno(i)),'g');
    axis tight
    xlabel('Time steps');
    ylabel(graphDynamicsYLabel(i));
  end
  drawnow;
end

% 3. Save data
filename = [basename num2str(j) '_H' num2str(H)]; save(filename);
