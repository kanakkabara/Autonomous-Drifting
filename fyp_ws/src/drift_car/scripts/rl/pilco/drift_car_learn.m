%% cartPole_learn.m
% *Summary:* Script to learn a controller for the cart-pole swingup
%
% Copyright (C) 2008-2013 by
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
% Last modified: 2013-03-27
%
%% High-Level Steps
% # Load parameters
% # Create J initial trajectories by applying random controls
% # Controlled learning (train dynamics model, policy learning, policy
% application)

%% Code

% 1. Initialization
rosshutdown;
rosinit;
plant.actionPub = rospublisher('/drift_car/action', 'std_msgs/Float64MultiArray');
plant.stateSub = rossubscriber('/drift_car/state');

clear all; close all;
settings_drift_car;                     % load scenario-specific settings
basename = 'driftCar_';                 % filename used for saving data

% 2. Initial J random rollouts
for jj = 1:J
  [xx, yy, realCost{jj}, latent{jj}] = ...
    rollout(gaussian(mu0, S0), struct('maxU',policy.maxU), H, plant, cost);
  x = [x; xx]; y = [y; yy];             % augment training sets for dynamics model  
end

filename = ['after_random_rollout']; save(filename);
plant.randomRollout = 1;          

mu0Sim(odei,:) = mu0; S0Sim(odei,odei) = S0;
mu0Sim = mu0Sim(dyno); S0Sim = S0Sim(dyno,dyno);

% 3. Controlled learning (N iterations)
for j = 1:N
  trainDynModel;                        % train (GP) dynamics model
  learnPolicy;                          % learn policy
  applyController;
  disp(['controlled trial # ' num2str(j)]);
end
