clear all; close all; clc; format compact;
run('loadParams.m');
addpath('../');

% intialize ros node
if(~robotics.ros.internal.Global.isNodeActive)
    rosinit;
end

detect_artag = 1;

if detect_artag
    ARTagSubscriber = rossubscriber('/tag_detections');
end

% Publishers
stateEstimatePublisher = rospublisher('/stateEstimate', 'terpcopter_msgs/stateEstimate');

stateMsg = rosmessage(stateEstimatePublisher);

if detect_artag
    ARTagMsg = ARTagSubscriber.LatestMessage;
end

r = robotics.Rate(100);
reset(r);

while(1)
    if detect_artag
        ARTagMsg = ARTagSubscriber.LatestMessage
    end
    
    if detect_artag
        ARTagsec = ARTagMsg.Header.Stamp.Sec; % raw format
        if(isempty(ARTagMsg.Detections))
            stateMsg.ArtagPresent = 0
            stateMsg.ArtagId = 0;
            stateMsg.ArtagPos = [0;0;0];
            stateMsg.ArtagPose = [0;0;0;0];
            
        else
            stateMsg.ArtagPresent = 1
%             stateMsg.ArtagId =  ARTagMsg.Detections.Id
%             stateMsg.ArtagPos = ARTagMsg.Detections.pose.pose.pose.position
%             stateMsg.ArtagPose = ARTagMsg.Detections.pose.pose.pose.orientation
            
        end
    end
    
    % fixed loop pause
    waitfor(r);
    
    % publish stateEstimate
    send(stateEstimatePublisher, stateMsg);
    disp('Sending stateMsg:')
    stateMsg
end
