% https://www.mathworks.com/help/nnet/ref/newrb.html
% https://www.mathworks.com/help/nnet/examples/radial-basis-approximation.html
% P = [1 2 3];
% T = [[2.0; 1] [4.1; 2] [5.9; 3]];
% net = newrb(P,T);
% 
% P = 1.5;
% Y = sim(net,P)

clear all; close all;
settings_drift_car;   
aug_states = [];
actions = [];
for i = 1:50000
    msg = receive(plant.stateSub);
    s = msg[1: end-2]; 
    % Augmented next states.
    aug_states(i,:) = gTrig(s, zeros(length(s)), angi);
    % Action taken that resulted in the state. (Steering angle in rads)
    actions(i) = msg[end]; 
end
 
disp("Received the data from the car");
disp("Displaying first few augmented states...");
disp(aug_states(:3));

aug_states(2:end, :) = aug_states(2:end, :) - aug_states(1: end - 1, :);

states_to_train = aug_states(2:end)';
action_to_train = actions(2:end)';

disp("Computed differences in states");
disp("First few state differences and actions...");
disp("States...");
disp(states_to_train(:3));
disp("Corresponding actions...");
disp(action_to_train(:3));


disp("Training initial policy with RBF...");
eg = 0;
sc = 1;
net = newrb(states_to_train, actions_to_train, eg, sc);
disp("Done training RBF network...");

rbf_center = cell2mat(newrb.IW);
length_scales = cell2mat(newrb.b)(1:end-1);
psuedo_targets = net(rbf_center);




% [xx, yy, realCost{1}, latent{1}] = ...
%     rollout(gaussian(mu0, S0), struct('maxU',policy.maxU), H, plant, cost);
% x = [x; xx]; y = [y; yy];       % augment training sets for dynamics model  

% actions = xx(:,9)';
% actions = xx';
% states = yy';
% 
% eg = 0; % sum-squared error goal
% sc = 1;    % spread constant
% net = newrb(actions, states, eg, sc);
% disp(net);
% 
% Y = net(actions(:,2))
% disp("Actual")
% disp(states(:,2))