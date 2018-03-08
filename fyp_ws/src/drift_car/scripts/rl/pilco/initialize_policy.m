% https://www.mathworks.com/help/nnet/ref/newrb.html
% https://www.mathworks.com/help/nnet/examples/radial-basis-approximation.html
% P = [1 2 3];
% T = [[2.0; 1] [4.1; 2] [5.9; 3]];
% net = newrb(P,T);
% 
% P = 1.5;
% Y = sim(net,P)
function [rbf_center, pseudo_targets, length_scales] = initialize_policy()
% settings_drift_car;
aug_states = [];
actions = [];
for i = 1:100
    msg = receive(plant.stateSub);
    msg = msg.Data;
    s = msg(1: end-2); 
    
    % Augmented next states.
    sa = gTrig(s, zeros(length(s)), angi);  %Augment angles
    temp = [s' sa'];              
    aug_states(i,:) = temp(poli);
    
    % Action taken that resulted in the state. (Steering angle in rads)
    actions(i) = msg(end); 
end
 
disp("Received the data from the car");
disp("Displaying first few augmented states...");

disp("Training initial policy with RBF...");
eg = 0;
sc = 1;
maxNeurons = 10;
net = newrb(aug_states', actions, eg, sc, maxNeurons);
disp("Done training RBF network...");
disp("Printing RBF network...");

rbf_center = cell2mat(net.IW);
biases = cell2mat(net.b);
length_scales = biases(1:end-1);
pseudo_targets = net(rbf_center');

disp("Pseudo Targets: ");
disp(pseudo_targets);

filename = ["drift_car_init"]; 
save(filename);


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