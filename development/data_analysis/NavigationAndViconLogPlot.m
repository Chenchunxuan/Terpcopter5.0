% Plotting both the Realsense VIO and the Motion Capture 
clear;
close all;
clc;

%% Toggle Flags
plotLidar = 1;
plotVIO = 1;
plotLocalPosition = 1;
plotStateEstimate = 0;
plotVicon = 1;

%% Correcting Offset
% If the Vicon initial takeoff is 3 seconds ahead of the local position
% takeoff, then the ViconTimeOffset should be -3 seconds.
% 
% ex. ViconTimeOffset = -3.1; shifts the vicon data back 3.1 seconds
%     ViconTimeOffset = 12.9; shifts the vicon data forward 12.9 seconds
ViconTimeOffset = -3.0; 

%% Load Navigation csv files
pathToNavigationLogs = '/home/amav/Terpcopter5.0/development/data_analysis/Navigation';
cd(pathToNavigationLogs)

% State Estimation file SELECT This Only
[file1,path1] = uigetfile('*.log');
checkString = extractBetween(file1,1,13);
isStateEstimate = strcmp(checkString{1,1}, 'stateEstimate');

if ~isStateEstimate
    disp('You must choose a stateEstimate csv log');
    return
end

% Lidar Log file
if plotLidar
    file2 = replaceBetween(file1,1,13,'lidar');
    path2 = path1;
end

% Local Position file
if plotLocalPosition
    file3 = replaceBetween(file1,1,13,'localPosition');
    path3 = path1;
end

% VIO file
if plotVIO
    file4 = replaceBetween(file1,1,13,'VIO');
    path4 = path1;
end

%% Load Vicon csv file
if plotVicon
    pathToViconLogs = '/home/amav/Terpcopter5.0/development/data_analysis/AMAV_Vicon/ViconVIOLidarLogs';
    cd(pathToViconLogs)
    
    [file5,path5] = uigetfile('*.log');
end


%% State Estimate
file1
filepath1 = [path1 file1];
data1 = csvread(filepath1);

stateEstimateYaw = data1(:,1);
stateEstimatePitch = data1(:,2);
stateEstimateRoll = data1(:,3);

stateEstimateUp = data1(:,6);

%% Lidar
if plotLidar
    file2
    filepath2 = [path2 file2];
    data2 = csvread(filepath2);
    
    PositionZLidar = data2(:,1);
end

%% Local Position
if plotLocalPosition
    file3
    filepath3 = [path3 file3];
    data3 = csvread(filepath3);
    
    % parse out
    TimeLocalPosition = data3(:,1);
    TimeLocalPosition = TimeLocalPosition - TimeLocalPosition(1);
    PositionXLocalPosition = data3(:,2);
    PositionYLocalPosition = data3(:,3);
    PositionZLocalPosition = data3(:,4);
    
    OrientationPhiLocalPosition = data3(:,5);
    OrientationThetaLocalPosition = data3(:,6);
    OrientationPsiLocalPosition = data3(:,7);
    
    LinearVelocityXLocalPosition = -data3(:,8);
    LinearVelocityYLocalPosition = -data3(:,9);
    LinearVelocityZLocalPosition = data3(:,10);
    AngularVelocityXLocalPosition = data3(:,11);
    AngularVelocityYLocalPosition = data3(:,12);
    AngularVelocityZLocalPosition = data3(:,13);
end

%% Realsense VIO
if plotVIO
    file4
    filepath4 = [path4 file4];
    data4 = csvread(filepath4);
    
    % parse out
    TimeVIO = data4(:,1);
    TimeVIO = TimeVIO - TimeVIO(1);
    PositionXVIO = data4(:,2);
    PositionYVIO = data4(:,3);
    PositionZVIO = data4(:,4);
    PositionXVIOBodyFrame = -PositionYVIO;
    PositionYVIOBodyFrame = PositionXVIO;
    PositionZVIOBodyFrame = PositionZVIO;
    
    OrientationPhiVIO = data4(:,5);
    OrientationThetaVIO = data4(:,6);
    OrientationPsiVIO = data4(:,7);
    
    LinearVelocityXVIO = -data4(:,8);
    LinearVelocityYVIO = -data4(:,9);
    LinearVelocityZVIO = data4(:,10);
    AngularVelocityXVIO = data4(:,11);
    AngularVelocityYVIO = data4(:,12);
    AngularVelocityZVIO = data4(:,13);
end

%% Vicon Motion Capture
if plotVicon
    file5
    filepath5 = [path5 file5];
    data5 = csvread(filepath5);
    
    % parse out
    SubjectIndex = data5(:,1);
    
    TimeVicon = data5(:,2);
    TimeVicon = TimeVicon + ViconTimeOffset;
    PositionXVicon = data5(:,3)/1000;
    PositionYVicon = data5(:,4)/1000;
    PositionZVicon = data5(:,5)/1000;
    
    OrientationPhiVicon = data5(:,6);
    OrientationThetaVicon = data5(:,7);
    OrientationPsiVicon = data5(:,8);
end

%%
figure(4)
hold on
if plotLidar
    plot(TimeLocalPosition, PositionZLidar);
end
if plotLocalPosition
    plot(TimeLocalPosition,PositionZLocalPosition);
end
if plotVIO
    plot(TimeVIO, PositionZVIOBodyFrame);
end
if plotVicon
    plot(TimeVicon, PositionZVicon);
end

xlabel('Time (seconds)');
ylabel('Position Z (meters)');
title('Position Z vs Time Comparision for Local Position, Realsense VIO, and Lidar')
legend('Lidar','Local Position (EKF)','Realsense VIO');
grid on
set(gca, 'FontSize', 12);
hold off