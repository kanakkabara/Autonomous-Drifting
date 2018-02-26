% P = [1 2 3];
% T = [[2.0; 1] [4.1; 2] [5.9; 3]];
% net = newrb(P,T);
% 
% P = 1.5;
% Y = sim(net,P)

clear all; close all;
settings_drift_car;   

[xx, yy, realCost{1}, latent{1}] = ...
    rollout(gaussian(mu0, S0), struct('maxU',policy.maxU), H, plant, cost);
x = [x; xx]; y = [y; yy];       % augment training sets for dynamics model  


% actions = xx(:,9)';
actions = xx';
states = yy';

eg = 0; % sum-squared error goal
sc = 1;    % spread constant
net = newrb(actions, states, eg, sc);

Y = net(actions(:,2))
disp("Actual")
disp(states(:,2))