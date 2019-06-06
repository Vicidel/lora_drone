%%
clear all; close all;

% open bag
bag = rosbag('./drone_logs/2019-06-06-10-56-54(R_went_too_far).bag');

% see info
rosbag info './drone_logs/2019-06-06-10-56-54(R_went_too_far).bag'

% select topic and read things
topic = select(bag, 'Topic', '/mavros/setpoint_raw/local');         % position target
setpoint_messages = readMessages(topic, 'DataFormat', 'struct');
topic = select(bag, 'Topic', '/mavros/local_position/pose');        % current xy position
position_messages = readMessages(topic, 'DataFormat', 'struct');


%%
% plot
figure; grid on; hold on;
for i=1: 10: length(setpoint_messages)
    plot(i, setpoint_messages{i}.Position.X, 'o'); 
end

% plot
figure; grid on; hold on;
for i=1: 10: length(position_messages)
    plot(i, position_messages{i}.Pose.Position.X, 'o'); 
end


