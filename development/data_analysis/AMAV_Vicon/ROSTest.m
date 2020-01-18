clear all; close all; clc;
rosinit('http://192.168.1.108:11311');

imuSubscriber = rossubscriber('/mavros/imu/data', 'sensor_msgs/Imu');

imuMsg = imuSubscriber.LatestMessage;

pause(2)

% r = robotics.Rate(100);
% reset(r);

while(1)
    
    imuMsg = imuSubscriber.LatestMessage
    
%     waitfor(r);
end