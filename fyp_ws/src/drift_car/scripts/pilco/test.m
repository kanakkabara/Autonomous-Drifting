actionPub = rospublisher('/matlab_bridge/action', 'std_msgs/Int8');
stateSub = rossubscriber('/matlab_bridge/state', @callback);

pause(2);

while 1
    actionMsg = rosmessage(actionPub);
    actionMsg.Data = 2;
    send(actionPub,actionMsg);
    pause(2);
end

function ws = callback(~, event) 
disp(event.Data)
ws = 0;
end
