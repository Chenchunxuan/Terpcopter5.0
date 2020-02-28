%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Node: Navigation
%
% Purpose:
% The purpose of the estimation node is to compute an estimate of the
% quadcopters state from noisy sensor data. This may include fusing data
% from different sources (e.g., barometer and lidar for altittude),
% filtering noisy signals (e.g., low-pass filters), implementing
% state estimators (e.g., kalman filters) and navigation algorithms.
%
% Input:
%   - ROS topics: several sensor data topics
%           /mavros/imu/data
%           /mavros/distance_sensor/hrlv_ez4_pub
%           /camera/odom/sample
%
%   - ROS topic: /features (generated by vision)
%
% Output:
%   - ROS topic: /stateEstimate (used by control, autonomy, vision, planning)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% prepare workspace
clear all; close all; clc; format compact;
run('loadParams.m');
addpath('../');

%run('updatePaths.m');
fprintf('Navigation Node Launching...\n');

% intialize ros node
if(~robotics.ros.internal.Global.isNodeActive)
    rosinit;
end

% toggleFlags
useLidarFlag = 1;
useVIOFlag = 1;

% Subscribers
imuDataSubscriber = rossubscriber('/mavros/imu/data');
localPositionOdomSubscriber = rossubscriber('/mavros/local_position/odom', 'nav_msgs/Odometry');
if useLidarFlag
    lidarDataSubscriber = rossubscriber('/mavros/distance_sensor/hrlv_ez4_pub');
end
if useVIOFlag
    VIODataSubscriber = rossubscriber('/camera/odom/sample', 'nav_msgs/Odometry');
end


% Publishers
stateEstimatePublisher = rospublisher('/stateEstimate', 'terpcopter_msgs/stateEstimate');

stateMsg = rosmessage(stateEstimatePublisher);

pause(2)
t0 = [];
t0_log = [];
t0_memory = [];

r = robotics.Rate(100);
reset(r);

% Receive Latest Imu and Lidar data
imuMsg = imuDataSubscriber.LatestMessage;
if useLidarFlag
    lidarMsg = lidarDataSubscriber.LatestMessage;
end
if useVIOFlag
    VIOMsg = VIODataSubscriber.LatestMessage
end
localPositionOdomMsg = localPositionOdomSubscriber.LatestMessage;

%% Pixhawk IMU
w = imuMsg.Orientation.W;
x = imuMsg.Orientation.X;
y = imuMsg.Orientation.Y;
z = imuMsg.Orientation.Z;

euler = quat2eul([w x y z]);

disp('psi:')
state.psi_inertial = rad2deg(euler(1))
state.theta = rad2deg(euler(2));
state.phi = rad2deg(euler(3));

%get relative yaw = - inertial yaw_intial - inertial yaw
inertial_yaw_initial = state.psi_inertial;

%% VIO Odometry
if useVIOFlag
    
    % VIO Time
    VIOTime = VIOMsg.Header.Stamp.Sec;
    VIOTimeNSec = VIOMsg.Header.Stamp.Nsec;
    % VIO Pose
    % Position
    VIOPositionX = VIOMsg.Pose.Pose.Position.X;
    VIOPositionY = VIOMsg.Pose.Pose.Position.Y;
    VIOPositionZ = VIOMsg.Pose.Pose.Position.Z;
    
    % Orientation
    VIOOrientationX = VIOMsg.Pose.Pose.Orientation.X;
    VIOOrientationY = VIOMsg.Pose.Pose.Orientation.Y;
    VIOOrientationZ = VIOMsg.Pose.Pose.Orientation.Z;
    VIOOrientationW = VIOMsg.Pose.Pose.Orientation.W;
    
    VIOeuler = quat2eul([VIOOrientationW VIOOrientationX VIOOrientationY VIOOrientationZ]);
    
    VIOpsi = rad2deg(VIOeuler(1));
    VIOtheta = rad2deg(VIOeuler(2));
    VIOphi = rad2deg(VIOeuler(3));

end
%% Local Position Odometry
% Local Position Time
localPositionTime = localPositionOdomMsg.Header.Stamp.Sec;
localPositionTimeNSec = localPositionOdomMsg.Header.Stamp.Nsec;
localPositionTimeLog = double(localPositionTime) + double(localPositionTimeNSec)*10^-9;
% Local Position Pose
% Position
localPositionX = localPositionOdomMsg.Pose.Pose.Position.X;
localPositionY = localPositionOdomMsg.Pose.Pose.Position.Y;
localPositionZ = localPositionOdomMsg.Pose.Pose.Position.Z;

% Orientation
localOrientationX = localPositionOdomMsg.Pose.Pose.Orientation.X;
localOrientationY = localPositionOdomMsg.Pose.Pose.Orientation.Y;
localOrientationZ = localPositionOdomMsg.Pose.Pose.Orientation.Z;
localOrientationW = localPositionOdomMsg.Pose.Pose.Orientation.W;

localPositionEuler = quat2eul([localOrientationW localOrientationX localOrientationY localOrientationZ]);

localPositionPsi = rad2deg(localPositionEuler(1));
localPositionTheta = rad2deg(localPositionEuler(2));
localPositionPhi = rad2deg(localPositionEuler(3));

%% Loggin information
dateString = datestr(now,'mmmm_dd_yyyy_HH_MM_SS_FFF');
if useVIOFlag
    VIOLog = ['/home/amav/Terpcopter5.0/development/data_analysis/Navigation' '/VIO_' dateString '.log'];
end
localPositionLog = ['/home/amav/Terpcopter5.0/development/data_analysis/Navigation' '/localPosition_' dateString '.log'];
if useLidarFlag
    lidarLog = ['/home/amav/Terpcopter5.0/development/data_analysis/Navigation' '/lidar_' dateString '.log'];
end
stateEstimateLog = ['/home/amav/Terpcopter5.0/development/data_analysis/Navigation' '/stateEstimate_' dateString '.log'];


while(1)
    tic
    pause(eps);
    % Receive Latest Imu and Lidar data
    imuMsg = imuDataSubscriber.LatestMessage;
    if useLidarFlag
        lidarMsg = lidarDataSubscriber.LatestMessage;
    end
    if useVIOFlag
        VIOMsg = VIODataSubscriber.LatestMessage;
    end
    localPositionOdomMsg = localPositionOdomSubscriber.LatestMessage;
    
    %% Pixhawk IMU
    if isempty(imuMsg)
        state = NaN;
        disp('No imu data\n');
        return;
    end
    w = imuMsg.Orientation.W;
    x = imuMsg.Orientation.X;
    y = imuMsg.Orientation.Y;
    z = imuMsg.Orientation.Z;
    
    euler = quat2eul([w x y z]);
    %yaw measured clock wise is negative.
    state.psi_inertial = mod(90-rad2deg(euler(1)),360);
    state.theta = -rad2deg(euler(2));
    state.phi = rad2deg(euler(3));
    state.quat = [x;y;z;w];
    %state.psi_inertial = round(state.psi_inertial,1);
    
    %get relative yaw = - inertial yaw_intial - inertial yaw
    if isempty(inertial_yaw_initial), inertial_yaw_initial = state.psi_inertial; end
    state.psi_relative = -state.psi_inertial + inertial_yaw_initial;
    
    %rounding off angles to 1 decimal place
    %state.psi_inertial = round(state.psi_inertial,1);
    state.psi_relative = round(state.psi_relative,1);
    state.theta = round(state.theta,1);
    state.phi = round(state.phi,1);
    
    %yaw lies between [-180 +180];
    if state.psi_relative> 180, state.psi_relative = state.psi_relative-360;
    elseif state.psi_relative<-180, state.psi_relative = 360+state.psi_relative;end
    
    stateMsg.Yaw = state.psi_relative;
    stateMsg.Roll = state.phi;
    stateMsg.Pitch = state.theta;
    stateMsg.Quat = state.quat;
    %% Lidar smoothing
    
    %% VIO Odometry
    if useVIOFlag
        
        % VIO Time
        VIOTime = VIOMsg.Header.Stamp.Sec;
        VIOTimeNSec = VIOMsg.Header.Stamp.Nsec;
        VIOTimeLog = double(VIOTime) + double(VIOTimeNSec)*10^-9;
        % VIO Pose
        % Position
        VIOPositionX = VIOMsg.Pose.Pose.Position.X;
        VIOPositionY = VIOMsg.Pose.Pose.Position.Y;
        VIOPositionZ = VIOMsg.Pose.Pose.Position.Z;
        
        % Orientation
        VIOOrientationX = VIOMsg.Pose.Pose.Orientation.X;
        VIOOrientationY = VIOMsg.Pose.Pose.Orientation.Y;
        VIOOrientationZ = VIOMsg.Pose.Pose.Orientation.Z;
        VIOOrientationW = VIOMsg.Pose.Pose.Orientation.W;
        
        VIOeuler = quat2eul([VIOOrientationW VIOOrientationX VIOOrientationY VIOOrientationZ]);
        
        VIOpsi = rad2deg(VIOeuler(1));
        VIOtheta = rad2deg(VIOeuler(2));
        VIOphi = rad2deg(VIOeuler(3));
        
        % VIO Twist
        VIOTwistLinearVelocityX = VIOMsg.Twist.Twist.Linear.X;
        VIOTwistLinearVelocityY = VIOMsg.Twist.Twist.Linear.Y;
        VIOTwistLinearVelocityZ = VIOMsg.Twist.Twist.Linear.Z;
        
        VIOTwistAngularVelocityX = VIOMsg.Twist.Twist.Angular.X;
        VIOTwistAngularVelocityY = VIOMsg.Twist.Twist.Angular.Y;
        VIOTwistAngularVelocityZ = VIOMsg.Twist.Twist.Angular.Z;
        
    end
    %% Local Position Odometry
    % Local Position Time
    localPositionTime = localPositionOdomMsg.Header.Stamp.Sec;
    localPositionTimeNSec = localPositionOdomMsg.Header.Stamp.Nsec;
    localPositionTimeLog = double(localPositionTime) + double(localPositionTimeNSec)*10^-9;
    % Local Position Pose
    % Position
    localPositionX = localPositionOdomMsg.Pose.Pose.Position.X;
    localPositionY = localPositionOdomMsg.Pose.Pose.Position.Y;
    localPositionZ = localPositionOdomMsg.Pose.Pose.Position.Z;
    stateMsg.East = localPositionX;
    stateMsg.North = localPositionY;
    stateMsg.Up = localPositionZ;
    
    % Orientation
    localOrientationX = localPositionOdomMsg.Pose.Pose.Orientation.X;
    localOrientationY = localPositionOdomMsg.Pose.Pose.Orientation.Y;
    localOrientationZ = localPositionOdomMsg.Pose.Pose.Orientation.Z;
    localOrientationW = localPositionOdomMsg.Pose.Pose.Orientation.W;
    
    localPositionEuler = quat2eul([localOrientationW localOrientationX localOrientationY localOrientationZ]);
    
    localPositionPsi = rad2deg(localPositionEuler(1));
    localPositionTheta = rad2deg(localPositionEuler(2));
    localPositionPhi = rad2deg(localPositionEuler(3));
    
    % Local Postion Twist
    localPositionTwistLinearVelocityX = localPositionOdomMsg.Twist.Twist.Linear.X;
    localPositionTwistLinearVelocityY = localPositionOdomMsg.Twist.Twist.Linear.Y;
    localPositionTwistLinearVelocityZ = localPositionOdomMsg.Twist.Twist.Linear.Z;
    
    %     stateMsg.xVelocity = localPositionTwistLinearVelocityX;
    %     stateMsg.yVelocity = localPositionTwistLinearVelocityY;
    %     stateMsg.zVelocity = localPositionTwistLinearVelocityZ;
    
    localPositionTwistAngularVelocityX = localPositionOdomMsg.Twist.Twist.Angular.X;
    localPositionTwistAngularVelocityY = localPositionOdomMsg.Twist.Twist.Angular.Y;
    localPositionTwistAngularVelocityZ = localPositionOdomMsg.Twist.Twist.Angular.Z;

    %% Logging Data
    % Opening the csv log file
    pFile1 = fopen(localPositionLog, 'a');
    if useVIOFlag
        pFile2 = fopen(VIOLog, 'a');
    end
    if useLidarFlag
        pFile3 = fopen(lidarLog, 'a');
    end
    pFile4 = fopen(stateEstimateLog, 'a');
    
    
    % Writing the data into the csv log
    % write csv file Local Position
    fprintf(pFile1,'%6.6f,',localPositionTimeLog);
    
    fprintf(pFile1,'%6.6f,',localPositionX);
    fprintf(pFile1,'%6.6f,',localPositionY);
    fprintf(pFile1,'%6.6f,',localPositionZ);
    fprintf(pFile1,'%6.6f,',localPositionPhi);
    fprintf(pFile1,'%6.6f,',localPositionTheta);
    fprintf(pFile1,'%6.6f,',localPositionPsi);
    
    fprintf(pFile1,'%6.6f,',localPositionTwistLinearVelocityX);
    fprintf(pFile1,'%6.6f,',localPositionTwistLinearVelocityY);
    fprintf(pFile1,'%6.6f,',localPositionTwistLinearVelocityZ);
    fprintf(pFile1,'%6.6f,',localPositionTwistAngularVelocityX);
    fprintf(pFile1,'%6.6f,',localPositionTwistAngularVelocityY);
    fprintf(pFile1,'%6.6f\n',localPositionTwistAngularVelocityZ);
    
    if useVIOFlag
        % write csv file Realsense VIO
        fprintf(pFile2,'%6.6f,',VIOTimeLog);
        
        fprintf(pFile2,'%6.6f,',VIOPositionX);
        fprintf(pFile2,'%6.6f,',VIOPositionY);
        fprintf(pFile2,'%6.6f,',VIOPositionZ);
        fprintf(pFile2,'%6.6f,',VIOphi);
        fprintf(pFile2,'%6.6f,',VIOtheta);
        fprintf(pFile2,'%6.6f,',VIOpsi);
        
        fprintf(pFile2,'%6.6f,',VIOTwistLinearVelocityX);
        fprintf(pFile2,'%6.6f,',VIOTwistLinearVelocityY);
        fprintf(pFile2,'%6.6f,',VIOTwistLinearVelocityZ);
        fprintf(pFile2,'%6.6f,',VIOTwistAngularVelocityX);
        fprintf(pFile2,'%6.6f,',VIOTwistAngularVelocityY);
        fprintf(pFile2,'%6.6f\n',VIOTwistAngularVelocityZ);
    end
    
    if useLidarFlag
        % write csv file lidar
        fprintf(pFile3,'%6.6f\n', lidarMsg.Range_);
    end
    
    % write csv file stateEstimate
    fprintf(pFile4,'%6.6f,',stateMsg.Yaw);
    fprintf(pFile4,'%6.6f,',stateMsg.Pitch);
    fprintf(pFile4,'%6.6f,',stateMsg.Roll);
    fprintf(pFile4,'%6.6f,',stateMsg.East);        % X axis
    fprintf(pFile4,'%6.6f,',stateMsg.North);       % Y axis
    fprintf(pFile4,'%6.6f\n',stateMsg.Up);         % Z axis
    
    fclose(pFile1);
    if useVIOFlag
        fclose(pFile2);
    end
    if useLidarFlag
        fclose(pFile3);
    end
    fclose(pFile4);
    
    %%
    % timestamp
    ti= rostime('now');
    abs_t = eval([int2str(ti.Sec) '.' ...
        int2str(ti.Nsec)]);
    
    if isempty(t0), t0 = abs_t; end
    t = abs_t-t0;
    t1 = abs_t-t0;
    stateMsg.Time = t;
    % fixed loop pause
    waitfor(r);
    
    fprintf('Yaw :   %03.01f\n',stateMsg.Yaw);
    fprintf('Pitch : %03.01f\n',stateMsg.Pitch);
    fprintf('Roll :  %03.01f\n',stateMsg.Roll);
    fprintf('East (X) :  %03.01f\n',stateMsg.East);
    fprintf('North (Y) :  %03.01f\n',stateMsg.North);
    fprintf('Altitude (Z) :  %03.01f\n',stateMsg.Up);
    fprintf('Altitude (Z) :  %03.01f\n',stateMsg.Range);
    fprintf('LoopRate Hz: %03d\n',round(1/toc) )
    
    
    % publish stateEstimate
    send(stateEstimatePublisher, stateMsg);
    disp('Sending stateMsg:')
    stateMsg
end