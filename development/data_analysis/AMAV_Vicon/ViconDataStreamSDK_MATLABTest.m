% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Copyright (C) OMG Plc 2009.
% All rights reserved.  This software is protected by copyright
% law and international treaties.  No part of this software / document
% may be reproduced or distributed in any form or by any means,
% whether transiently or incidentally to some other use of this software,
% without the written permission of the copyright owner.
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Part of the Vicon DataStream SDK for MATLAB.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

% %% ROS 
% ROS_Master_ip = 'http://192.168.1.108:11311';
% 
% if(~robotics.ros.internal.Global.isNodeActive)
%     rosinit(ROS_Master_ip);
% end

% % Subscribers
% VIODataSubscriber = rossubscriber('/camera/odom/sample', 'nav_msgs/Odometry');
% LidarSubscriber = rossubscriber('/mavros/distance_sensor/hrlv_ez4_pub');
% 
% % Lastest Message
% VIOMsg = VIODataSubscriber.LatestMessage;
% LidarMsg = LidarSubscriber.LatestMessage;

r = robotics.Rate(100);
reset(r);

%% Logging Vicon Data Stream and ROS Messages

firstIteration = true;
% Setup Logging Location
dateString = datestr(now,'mmmm_dd_yyyy_HH_MM_SS_FFF');
ViconLog = ['C:\Users\Vicon User\Documents\AMAV\Terpcopter5.0\development\data_analysis\AMAV_Vicon\ViconVIOLidarLogs' '/Vicon_' dateString '.log'];
VIOLog = ['C:\Users\Vicon User\Documents\AMAV\Terpcopter5.0\development\data_analysis\AMAV_Vicon\ViconVIOLidarLogs' '/VIO_' dateString '.log'];
LidarLog = ['C:\Users\Vicon User\Documents\AMAV\Terpcopter5.0\development\data_analysis\AMAV_Vicon\ViconVIOLidarLogs' '/Lidar_' dateString '.log'];
LocalPositionLog = ['C:\Users\Vicon User\Documents\AMAV\Terpcopter5.0\development\data_analysis\AMAV_Vicon\ViconVIOLidarLogs' '/Local_Position_' dateString '.log'];

% Program options
TransmitMulticast = false;
EnableHapticFeedbackTest = false;
HapticOnList = {'ViconAP_001';'ViconAP_002'};
SubjectFilterApplied = false;

% Check whether these variables exist, as they can be set by the command line on launch
% If you run the script with Command Window in Matlab, these workspace vars could persist the value from previous runs even not set in the Command Window
% You could clear the value with "clearvars"
if ~exist( 'bReadCentroids' )
  bReadCentroids = false;
end

if ~exist( 'bReadRays' )
  bReadRays = false;
end

if ~exist( 'bTrajectoryIDs' )
  bTrajectoryIDs = false;
end

if ~exist( 'axisMapping' )
  axisMapping = 'ZUp';
end

% example for running from commandline in the ComandWindow in Matlab
% e.g. bLightweightSegment = true;HostName = 'localhost:801';ViconDataStreamSDK_MATLABTest
if ~exist('bLightweightSegment')
  bLightweightSegment = false;
end

% Pass the subjects to be filtered in
% e.g. Subject = {'Subject1'};HostName = 'localhost:801';ViconDataStreamSDK_MATLABTest
EnableSubjectFilter  = exist('subjects');

% A dialog to stop the loop
MessageBox = msgbox( 'Stop DataStream Client', 'Vicon DataStream SDK' );

% Load the SDK
fprintf( 'Loading SDK...' );
Client.LoadViconDataStreamSDK();
fprintf( 'done\n' );

% Program options
if ~exist( 'HostName' )
  HostName = 'localhost:801';
end

fprintf( 'Centroids Enabled: %s\n', mat2str( bReadCentroids ) );
fprintf( 'Rays Enabled: %s\n', mat2str( bReadRays ) );
fprintf( 'Trajectory IDs Enabled: %s\n', mat2str( bTrajectoryIDs ) );
fprintf( 'Lightweight Segment Data Enabled: %s\n', mat2str( bLightweightSegment ) );
fprintf('Axis Mapping: %s\n', axisMapping );

if exist('undefVar')
  fprintf('Undefined Variable: %s\n', mat2str( undefVar ) );
end

% Make a new client
MyClient = Client();

% Connect to a server
fprintf( 'Connecting to %s ...', HostName );
while ~MyClient.IsConnected().Connected
  % Direct connection
  MyClient.Connect( HostName );
  
  % Multicast connection
  % MyClient.ConnectToMulticast( HostName, '224.0.0.0' );
  
  fprintf( '.' );
end
fprintf( '\n' );

% Enable some different data types
MyClient.EnableSegmentData();
MyClient.EnableMarkerData();
MyClient.EnableUnlabeledMarkerData();
MyClient.EnableDeviceData();
if bReadCentroids
  MyClient.EnableCentroidData();
end
if bReadRays
  MyClient.EnableMarkerRayData();
end
if bLightweightSegment
  MyClient.DisableLightweightSegmentData();
  Output_EnableLightweightSegment = MyClient.EnableLightweightSegmentData();
  if Output_EnableLightweightSegment.Result.Value ~= Result.Success
    fprintf( 'Server does not support lightweight segment data.\n' );
  end
end

fprintf( 'Segment Data Enabled: %s\n',             AdaptBool( MyClient.IsSegmentDataEnabled().Enabled ) );
fprintf( 'Lightweight Segment Data Enabled: %s\n', AdaptBool( MyClient.IsLightweightSegmentDataEnabled().Enabled ) );
fprintf( 'Marker Data Enabled: %s\n',              AdaptBool( MyClient.IsMarkerDataEnabled().Enabled ) );
fprintf( 'Unlabeled Marker Data Enabled: %s\n',    AdaptBool( MyClient.IsUnlabeledMarkerDataEnabled().Enabled ) );
fprintf( 'Device Data Enabled: %s\n',              AdaptBool( MyClient.IsDeviceDataEnabled().Enabled ) );
fprintf( 'Centroid Data Enabled: %s\n',            AdaptBool( MyClient.IsCentroidDataEnabled().Enabled ) );
fprintf( 'Marker Ray Data Enabled: %s\n',          AdaptBool( MyClient.IsMarkerRayDataEnabled().Enabled ) );

MyClient.SetBufferSize(1);
% Set the streaming mode
MyClient.SetStreamMode( StreamMode.ClientPull );
% MyClient.SetStreamMode( StreamMode.ClientPullPreFetch );
% MyClient.SetStreamMode( StreamMode.ServerPush );

% Set the global up axis
if axisMapping == 'XUp'
  MyClient.SetAxisMapping( Direction.Up, ...
                          Direction.Forward,      ...
                          Direction.Left ); % X-up
elseif axisMapping == 'YUp'
  MyClient.SetAxisMapping( Direction.Forward, ...
                         Direction.Up,    ...
                         Direction.Right );    % Y-up
else
  MyClient.SetAxisMapping( Direction.Forward, ...
                         Direction.Left,    ...
                         Direction.Up );    % Z-up
end

Output_GetAxisMapping = MyClient.GetAxisMapping();
fprintf( 'Axis Mapping: X-%s Y-%s Z-%s\n', Output_GetAxisMapping.XAxis.ToString(), ...
                                           Output_GetAxisMapping.YAxis.ToString(), ...
                                           Output_GetAxisMapping.ZAxis.ToString() );


% Discover the version number
Output_GetVersion = MyClient.GetVersion();
fprintf( 'Version: %d.%d.%d.%d\n', Output_GetVersion.Major, ...
                                Output_GetVersion.Minor, ...
                                Output_GetVersion.Point, ...
                                Output_GetVersion.Revision );
  
if TransmitMulticast
  MyClient.StartTransmittingMulticast( 'localhost', '224.0.0.0' );
end  

t_start = datestr(now, 'HH:MM:SS.FFF'); % TEMPORARY
time_start = datevec(t_start, 'HH:MM:SS.FFF')
timeStartSec = time_start(4)*3600 + time_start(5)*60 + time_start(6);

Counter = 1;
% Loop until the message box is dismissed
while ishandle( MessageBox )
  drawnow;
  Counter = Counter + 1;
  
  % Get a frame
  fprintf( 'Waiting for new frame...' );
  while MyClient.GetFrame().Result.Value ~= Result.Success
    fprintf( '.' );
  end% while
  fprintf( '\n' );  
  
  if EnableHapticFeedbackTest
    if mod( Counter,2 ) == 0
      for i = 1:length( HapticOnList )
          DeviceName = HapticOnList{i};
          Output_GetApexFeedback = MyClient.SetApexDeviceFeedback( DeviceName, true );
          if Output_GetApexFeedback.Result.Value == Result.Success
              fprintf( 'Turn haptic feedback on for device: %s\n', DeviceName );
          elseif Output_GetApexFeedback.Result.Value == Result.InvalidDeviceName
              fprintf( 'Device doesn''t exist: %s\n', DeviceName );
          end
      end
    end
    if mod( Counter, 20 ) == 0
      for i = 1:length( HapticOnList )
          DeviceName = HapticOnList{i};
          Output_GetApexFeedback = MyClient.SetApexDeviceFeedback( DeviceName, false );
          if Output_GetApexFeedback.Result.Value == Result.Success
              fprintf( 'Turn haptic feedback on for device: %s\n', DeviceName );
          end
      end
    end
  end
      

  % Get the frame number
  Output_GetFrameNumber = MyClient.GetFrameNumber();
  fprintf( 'Frame Number: %d\n', Output_GetFrameNumber.FrameNumber );

  % Get the frame rate
  Output_GetFrameRate = MyClient.GetFrameRate();
  fprintf( 'Frame rate: %g\n', Output_GetFrameRate.FrameRateHz );

  for FrameRateIndex = 1:MyClient.GetFrameRateCount().Count
    FrameRateName  = MyClient.GetFrameRateName( FrameRateIndex ).Name;
    FrameRateValue = MyClient.GetFrameRateValue( FrameRateName ).Value;

    fprintf( '%s: %gHz\n', FrameRateName, FrameRateValue );
  end% for  

  fprintf( '\n' );
  % Get the timecode
  Output_GetTimecode = MyClient.GetTimecode();
  fprintf( 'Timecode: %dh %dm %ds %df %dsf %s %d %d %d\n\n',    ...
                     Output_GetTimecode.Hours,                  ...
                     Output_GetTimecode.Minutes,                ...
                     Output_GetTimecode.Seconds,                ...
                     Output_GetTimecode.Frames,                 ...
                     Output_GetTimecode.SubFrame,               ...
                     AdaptBool( Output_GetTimecode.FieldFlag ), ...
                     Output_GetTimecode.Standard.Value,         ...
                     Output_GetTimecode.SubFramesPerFrame,      ...
                     Output_GetTimecode.UserBits );

  % Get the latency
  fprintf( 'Latency: %gs\n', MyClient.GetLatencyTotal().Total );
  
  for LatencySampleIndex = 1:MyClient.GetLatencySampleCount().Count
    SampleName  = MyClient.GetLatencySampleName( LatencySampleIndex ).Name;
    SampleValue = MyClient.GetLatencySampleValue( SampleName ).Value;

    fprintf( '  %s %gs\n', SampleName, SampleValue );
  end% for  
  fprintf( '\n' );
                     
  if EnableSubjectFilter && ~SubjectFilterApplied 
    for SubjectIndex = 1: length( Subject )
      Output_SubjectFilter = MyClient.AddToSubjectFilter(char( Subject(SubjectIndex)));
      SubjectFilterApplied = SubjectFilterApplied || Output_SubjectFilter.Result.Value == Result.Success;
    end
  end
  
  % Count the number of subjects
  SubjectCount = MyClient.GetSubjectCount().SubjectCount;
  fprintf( 'Subjects (%d):\n', SubjectCount );
  for SubjectIndex = 1:SubjectCount
    fprintf( '  Subject #%d\n', SubjectIndex - 1 );
    
    % Get the subject name
    SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;
    fprintf( '    Name: %s\n', SubjectName );
    
    % Get the root segment
    RootSegment = MyClient.GetSubjectRootSegmentName( SubjectName ).SegmentName;
    fprintf( '    Root Segment: %s\n', RootSegment );

    % Count the number of segments
    SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;
    fprintf( '    Segments (%d):\n', SegmentCount );
    for SegmentIndex = 1:SegmentCount
      fprintf( '      Segment #%d\n', SegmentIndex - 1 );
      
      % Get the segment name
      SegmentName = MyClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;
      fprintf( '        Name: %s\n', SegmentName );
      
      % Get the segment parent
      SegmentParentName = MyClient.GetSegmentParentName( SubjectName, SegmentName ).SegmentName;
      fprintf( '        Parent: %s\n',  SegmentParentName );

      % Get the segment's children
      ChildCount = MyClient.GetSegmentChildCount( SubjectName, SegmentName ).SegmentCount;
      fprintf( '     Children (%d):\n', ChildCount );
      for ChildIndex = 1:ChildCount
        ChildName = MyClient.GetSegmentChildName( SubjectName, SegmentName, ChildIndex ).SegmentName;
        fprintf( '       %s\n', ChildName );
      end% for  

      % Get the static segment translation
      Output_GetSegmentStaticTranslation = MyClient.GetSegmentStaticTranslation( SubjectName, SegmentName );
      fprintf( '        Static Translation: (%g, %g, %g)\n',                  ...
                         Output_GetSegmentStaticTranslation.Translation( 1 ), ...
                         Output_GetSegmentStaticTranslation.Translation( 2 ), ...
                         Output_GetSegmentStaticTranslation.Translation( 3 ) );
      
      % Get the static segment rotation in helical co-ordinates
      Output_GetSegmentStaticRotationHelical = MyClient.GetSegmentStaticRotationHelical( SubjectName, SegmentName );
      fprintf( '        Static Rotation Helical: (%g, %g, %g)\n',              ...
                         Output_GetSegmentStaticRotationHelical.Rotation( 1 ), ...
                         Output_GetSegmentStaticRotationHelical.Rotation( 2 ), ...
                         Output_GetSegmentStaticRotationHelical.Rotation( 3 ) );
      
      % Get the static segment rotation as a matrix
      Output_GetSegmentStaticRotationMatrix = MyClient.GetSegmentStaticRotationMatrix( SubjectName, SegmentName );
      fprintf( '        Static Rotation Matrix: (%g, %g, %g, %g, %g, %g, %g, %g, %g)\n', ...
                         Output_GetSegmentStaticRotationMatrix.Rotation( 1 ),            ...
                         Output_GetSegmentStaticRotationMatrix.Rotation( 2 ),            ...
                         Output_GetSegmentStaticRotationMatrix.Rotation( 3 ),            ...
                         Output_GetSegmentStaticRotationMatrix.Rotation( 4 ),            ...
                         Output_GetSegmentStaticRotationMatrix.Rotation( 5 ),            ...
                         Output_GetSegmentStaticRotationMatrix.Rotation( 6 ),            ...
                         Output_GetSegmentStaticRotationMatrix.Rotation( 7 ),            ...
                         Output_GetSegmentStaticRotationMatrix.Rotation( 8 ),            ...
                         Output_GetSegmentStaticRotationMatrix.Rotation( 9 ) );
      
      % Get the static segment rotation in quaternion co-ordinates
      Output_GetSegmentStaticRotationQuaternion = MyClient.GetSegmentStaticRotationQuaternion( SubjectName, SegmentName );
      fprintf( '        Static Rotation Quaternion: (%g, %g, %g, %g)\n',          ...
                         Output_GetSegmentStaticRotationQuaternion.Rotation( 1 ), ...
                         Output_GetSegmentStaticRotationQuaternion.Rotation( 2 ), ...
                         Output_GetSegmentStaticRotationQuaternion.Rotation( 3 ), ...
                         Output_GetSegmentStaticRotationQuaternion.Rotation( 4 ) );

      % Get the static segment rotation in EulerXYZ co-ordinates
      Output_GetSegmentStaticRotationEulerXYZ = MyClient.GetSegmentStaticRotationEulerXYZ( SubjectName, SegmentName );
      fprintf( '        Static Rotation EulerXYZ: (%g, %g, %g)\n',               ...
                         Output_GetSegmentStaticRotationEulerXYZ.Rotation( 1 ),  ...
                         Output_GetSegmentStaticRotationEulerXYZ.Rotation( 2 ),  ...
                         Output_GetSegmentStaticRotationEulerXYZ.Rotation( 3 ) );

      % Get the global segment translation
      Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
      fprintf( '        Global Translation: (%g, %g, %g) %s\n',               ...
                         Output_GetSegmentGlobalTranslation.Translation( 1 ), ...
                         Output_GetSegmentGlobalTranslation.Translation( 2 ), ...
                         Output_GetSegmentGlobalTranslation.Translation( 3 ), ...
                         AdaptBool( Output_GetSegmentGlobalTranslation.Occluded ) );
      
      % Get the global segment rotation in helical co-ordinates
      Output_GetSegmentGlobalRotationHelical = MyClient.GetSegmentGlobalRotationHelical( SubjectName, SegmentName );
      fprintf( '        Global Rotation Helical: (%g, %g, %g) %s\n',           ...
                         Output_GetSegmentGlobalRotationHelical.Rotation( 1 ), ...
                         Output_GetSegmentGlobalRotationHelical.Rotation( 2 ), ...
                         Output_GetSegmentGlobalRotationHelical.Rotation( 3 ), ...
                         AdaptBool( Output_GetSegmentGlobalRotationHelical.Occluded ) );
      
      % Get the global segment rotation as a matrix
      Output_GetSegmentGlobalRotationMatrix = MyClient.GetSegmentGlobalRotationMatrix( SubjectName, SegmentName );
      fprintf( '        Global Rotation Matrix: (%g, %g, %g, %g, %g, %g, %g, %g, %g) %s\n', ...
                         Output_GetSegmentGlobalRotationMatrix.Rotation( 1 ),               ...
                         Output_GetSegmentGlobalRotationMatrix.Rotation( 2 ),               ...
                         Output_GetSegmentGlobalRotationMatrix.Rotation( 3 ),               ...
                         Output_GetSegmentGlobalRotationMatrix.Rotation( 4 ),               ...
                         Output_GetSegmentGlobalRotationMatrix.Rotation( 5 ),               ...
                         Output_GetSegmentGlobalRotationMatrix.Rotation( 6 ),               ...
                         Output_GetSegmentGlobalRotationMatrix.Rotation( 7 ),               ...
                         Output_GetSegmentGlobalRotationMatrix.Rotation( 8 ),               ...
                         Output_GetSegmentGlobalRotationMatrix.Rotation( 9 ),               ...
                         AdaptBool( Output_GetSegmentGlobalRotationMatrix.Occluded ) );
      
      % Get the global segment rotation in quaternion co-ordinates
      Output_GetSegmentGlobalRotationQuaternion = MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
      fprintf( '        Global Rotation Quaternion: (%g, %g, %g, %g) %s\n',             ...
                         Output_GetSegmentGlobalRotationQuaternion.Rotation( 1 ),       ...
                         Output_GetSegmentGlobalRotationQuaternion.Rotation( 2 ),       ...
                         Output_GetSegmentGlobalRotationQuaternion.Rotation( 3 ),       ...
                         Output_GetSegmentGlobalRotationQuaternion.Rotation( 4 ),       ...
                         AdaptBool( Output_GetSegmentGlobalRotationQuaternion.Occluded ) );

      % Get the global segment rotation in EulerXYZ co-ordinates
      Output_GetSegmentGlobalRotationEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ( SubjectName, SegmentName );
      fprintf( '        Global Rotation EulerXYZ: (%g, %g, %g) %s\n',                 ...
                         Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 1 ),       ...
                         Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 2 ),       ...
                         Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 3 ),       ...
                         AdaptBool( Output_GetSegmentGlobalRotationEulerXYZ.Occluded ) );

      % Get the local segment translation
      Output_GetSegmentLocalTranslation = MyClient.GetSegmentLocalTranslation( SubjectName, SegmentName );
      fprintf( '        Local Translation: (%g, %g, %g) %s\n',               ...
                         Output_GetSegmentLocalTranslation.Translation( 1 ), ...
                         Output_GetSegmentLocalTranslation.Translation( 2 ), ...
                         Output_GetSegmentLocalTranslation.Translation( 3 ), ...
                         AdaptBool( Output_GetSegmentLocalTranslation.Occluded ) );
      
      % Get the local segment rotation in helical co-ordinates
      Output_GetSegmentLocalRotationHelical = MyClient.GetSegmentLocalRotationHelical( SubjectName, SegmentName );
      fprintf( '        Local Rotation Helical: (%g, %g, %g) %s\n',           ...
                         Output_GetSegmentLocalRotationHelical.Rotation( 1 ), ...
                         Output_GetSegmentLocalRotationHelical.Rotation( 2 ), ...
                         Output_GetSegmentLocalRotationHelical.Rotation( 3 ), ...
                         AdaptBool( Output_GetSegmentLocalRotationHelical.Occluded ) );
      
      % Get the local segment rotation as a matrix
      Output_GetSegmentLocalRotationMatrix = MyClient.GetSegmentLocalRotationMatrix( SubjectName, SegmentName );
      fprintf( '        Local Rotation Matrix: (%g, %g, %g, %g, %g, %g, %g, %g, %g) %s\n', ...
                         Output_GetSegmentLocalRotationMatrix.Rotation( 1 ),               ...
                         Output_GetSegmentLocalRotationMatrix.Rotation( 2 ),               ...
                         Output_GetSegmentLocalRotationMatrix.Rotation( 3 ),               ...
                         Output_GetSegmentLocalRotationMatrix.Rotation( 4 ),               ...
                         Output_GetSegmentLocalRotationMatrix.Rotation( 5 ),               ...
                         Output_GetSegmentLocalRotationMatrix.Rotation( 6 ),               ...
                         Output_GetSegmentLocalRotationMatrix.Rotation( 7 ),               ...
                         Output_GetSegmentLocalRotationMatrix.Rotation( 8 ),               ...
                         Output_GetSegmentLocalRotationMatrix.Rotation( 9 ),               ...
                         AdaptBool( Output_GetSegmentLocalRotationMatrix.Occluded ) );
      
      % Get the local segment rotation in quaternion co-ordinates
      Output_GetSegmentLocalRotationQuaternion = MyClient.GetSegmentLocalRotationQuaternion( SubjectName, SegmentName );
      fprintf( '        Local Rotation Quaternion: (%g, %g, %g, %g) %s\n',             ...
                         Output_GetSegmentLocalRotationQuaternion.Rotation( 1 ),       ...
                         Output_GetSegmentLocalRotationQuaternion.Rotation( 2 ),       ...
                         Output_GetSegmentLocalRotationQuaternion.Rotation( 3 ),       ...
                         Output_GetSegmentLocalRotationQuaternion.Rotation( 4 ),       ...
                         AdaptBool( Output_GetSegmentLocalRotationQuaternion.Occluded ) );

      % Get the local segment rotation in EulerXYZ co-ordinates
      Output_GetSegmentLocalRotationEulerXYZ = MyClient.GetSegmentLocalRotationEulerXYZ( SubjectName, SegmentName );
      fprintf( '        Local Rotation EulerXYZ: (%g, %g, %g) %s\n',                 ...
                         Output_GetSegmentLocalRotationEulerXYZ.Rotation( 1 ),       ...
                         Output_GetSegmentLocalRotationEulerXYZ.Rotation( 2 ),       ...
                         Output_GetSegmentLocalRotationEulerXYZ.Rotation( 3 ),       ...
                         AdaptBool( Output_GetSegmentLocalRotationEulerXYZ.Occluded ) );

%       VIOMsg = VIODataSubscriber.LatestMessage
%       LidarMsg = LidarSubscriber.LatestMessage;
%       
%       % Intel Realsense VIO Pose
%       % Position
%       VIOPositionX = VIOMsg.Pose.Pose.Position.X;
%       VIOPositionY = VIOMsg.Pose.Pose.Position.Y;
%       VIOPositionZ = VIOMsg.Pose.Pose.Position.Z;
%       
%       % Orientation
%       VIOOrientationX = VIOMsg.Pose.Pose.Orientation.X;
%       VIOOrientationY = VIOMsg.Pose.Pose.Orientation.Y;
%       VIOOrientationZ = VIOMsg.Pose.Pose.Orientation.Z;
%       VIOOrientationW = VIOMsg.Pose.Pose.Orientation.W;
%       
%   
%       % Lidar
%       LidarRange = LidarMsg.Range;
      
      [pFile1, msg] = fopen(ViconLog, 'a');
%       [pFile2, msg] = fopen(VIOLog, 'a');
%       pFile3 = fopen(LidarLog, 'a');
      
      if pFile1 < 0
          error('Failed to open file "%s" for writing, because "%s"', ViconLog, msg);
      end
   
      X = Output_GetSegmentGlobalTranslation.Translation( 1 );
      Y = Output_GetSegmentGlobalTranslation.Translation( 2 );
      Z = Output_GetSegmentGlobalTranslation.Translation( 3 );
      phi = Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 1 );
      theta = Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 2 );
      psi = Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 3 );
      
      % write csv file for the Vicon Motion Capture
      t = datestr(now, 'HH:MM:SS.FFF'); % TEMPORARY
      time = datevec(t, 'HH:MM:SS.FFF')
      timeSec = time(4)*3600 + time(5)*60 + time(6);
      time = timeSec - timeStartSec;
      
      if(firstIteration)
          firstIteration = false;
          continue;
      else
          fprintf(pFile1, '%f,',SubjectIndex);
          fprintf(pFile1, '%6.6f,', time);
          fprintf(pFile1, '%6.6f,', X);
          fprintf(pFile1, '%6.6f,', Y);
          fprintf(pFile1, '%6.6f,', Z);
          fprintf(pFile1, '%6.6f,', phi);
          fprintf(pFile1, '%6.6f,', theta);
          fprintf(pFile1, '%6.6f\n', psi);
          
%           
%           fprintf(pFile2, '%6.6f,', VIOPositionX);
%           fprintf(pFile2, '%6.6f,', VIOPositionY);
%           fprintf(pFile2, '%6.6f\n', VIOPositionZ);
%           
%           fprintf(pFile3, '%6.6f\n', LidarRange);
      end
      
      fclose(pFile1);
%       fclose(pFile2);
%       fclose(pFile3);
    
    end% SegmentIndex

    % Get the quality of the subject (object) if supported
    Output_GetObjectQuality = MyClient.GetObjectQuality( SubjectName );
    if Output_GetObjectQuality.Result.Value == Result.Success
      fprintf( '    Quality: %g\n', Output_GetObjectQuality.Quality );
    end

    % Count the number of markers
    MarkerCount = MyClient.GetMarkerCount( SubjectName ).MarkerCount;
    fprintf( '    Markers (%d):\n', MarkerCount );
    for MarkerIndex = 1:MarkerCount
      % Get the marker name
      MarkerName = MyClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;

      % Get the marker parent
      MarkerParentName = MyClient.GetMarkerParentName( SubjectName, MarkerName ).SegmentName;

      % Get the global marker translation
      Output_GetMarkerGlobalTranslation = MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );

      fprintf( '      Marker #%d: %s (%g, %g, %g) %s\n',                     ...
                         MarkerIndex - 1,                                    ...
                         MarkerName,                                         ...
                         Output_GetMarkerGlobalTranslation.Translation( 1 ), ...
                         Output_GetMarkerGlobalTranslation.Translation( 2 ), ...
                         Output_GetMarkerGlobalTranslation.Translation( 3 ), ...
                         AdaptBool( Output_GetMarkerGlobalTranslation.Occluded ) );

      if bReadRays
        % Get the ray contributions for this marker
        Output_GetMarkerRayContributionCount = MyClient.GetMarkerRayContributionCount( SubjectName, MarkerName );
        if( Output_GetMarkerRayContributionCount.Result.Value == Result.Success )
          fprintf('      Contributed to by: ');

		  MarkerRayContributionCount = Output_GetMarkerRayContributionCount.RayContributionsCount;
          for ContributionIndex = 1: MarkerRayContributionCount
            Output_GetMarkerRayContribution = MyClient.GetMarkerRayContribution(SubjectName, MarkerName, ContributionIndex);
            fprintf( 'ID:%d Index:%d ', Output_GetMarkerRayContribution.CameraID, Output_GetMarkerRayContribution.CentroidIndex);
          end

          fprintf('\n' );
        end
      end% bReadRays
    end% MarkerIndex
    
  end% SubjectIndex

  % Get the unlabeled markers
  UnlabeledMarkerCount = MyClient.GetUnlabeledMarkerCount().MarkerCount;
  fprintf( '  Unlabeled Markers (%d):\n', UnlabeledMarkerCount );
  for UnlabeledMarkerIndex = 1:UnlabeledMarkerCount
    % Get the global marker translation
    Output_GetUnlabeledMarkerGlobalTranslation = MyClient.GetUnlabeledMarkerGlobalTranslation( UnlabeledMarkerIndex );
    if bTrajectoryIDs
      fprintf( '    Marker #%d: (%g, %g, %g): Traj ID %d\n',                                    ...
                         UnlabeledMarkerIndex - 1,                                    ...
                         Output_GetUnlabeledMarkerGlobalTranslation.Translation( 1 ), ...
                         Output_GetUnlabeledMarkerGlobalTranslation.Translation( 2 ), ...
                         Output_GetUnlabeledMarkerGlobalTranslation.Translation( 3 ), ...
                         Output_GetUnlabeledMarkerGlobalTranslation.MarkerID );
    else
      fprintf( '    Marker #%d: (%g, %g, %g)\n',                                    ...
                         UnlabeledMarkerIndex - 1,                                    ...
                         Output_GetUnlabeledMarkerGlobalTranslation.Translation( 1 ), ...
                         Output_GetUnlabeledMarkerGlobalTranslation.Translation( 2 ), ...
                         Output_GetUnlabeledMarkerGlobalTranslation.Translation( 3 ) );
    end
  end% UnlabeledMarkerIndex

  % Get the labeled markers
  LabeledMarkerCount = MyClient.GetLabeledMarkerCount().MarkerCount;
  fprintf( '  Labeled Markers (%d):\n', LabeledMarkerCount );
  for LabeledMarkerIndex = 1:LabeledMarkerCount
    % Get the global marker translation
    Output_GetLabeledMarkerGlobalTranslation = MyClient.GetLabeledMarkerGlobalTranslation( LabeledMarkerIndex );
    if bTrajectoryIDs
      fprintf( '    Marker #%d: (%g, %g, %g): Traj ID %d\n',                                  ...
                         LabeledMarkerIndex - 1,                                    ...
                         Output_GetLabeledMarkerGlobalTranslation.Translation( 1 ), ...
                         Output_GetLabeledMarkerGlobalTranslation.Translation( 2 ), ...
                         Output_GetLabeledMarkerGlobalTranslation.Translation( 3 ), ...
                         Output_GetLabeledMarkerGlobalTranslation.MarkerID );
    else
      fprintf( '    Marker #%d: (%g, %g, %g)\n',                                  ...
                         LabeledMarkerIndex - 1,                                    ...
                         Output_GetLabeledMarkerGlobalTranslation.Translation( 1 ), ...
                         Output_GetLabeledMarkerGlobalTranslation.Translation( 2 ), ...
                         Output_GetLabeledMarkerGlobalTranslation.Translation( 3 ) );
    end
  end% LabeledMarkerIndex
  if bReadCentroids
    CameraCount = MyClient.GetCameraCount().CameraCount;
    fprintf( 'Cameras(%d):\n', CameraCount);

    for CameraIndex = 1:CameraCount
      fprintf('  Camera #%d:\n', CameraIndex - 1 );
        
      CameraName = MyClient.GetCameraName( CameraIndex ).CameraName;
      fprintf ( '    Name: %s\n', CameraName );

      fprintf( '    Id: %d\n', MyClient.GetCameraId( CameraName ).CameraId );
      fprintf( '    User Id: %d\n' , MyClient.GetCameraUserId( CameraName ).CameraUserId );
      fprintf('    Type: %s\n', MyClient.GetCameraType( CameraName ).CameraType);
      fprintf('    Display Name: %s\n', MyClient.GetCameraDisplayName( CameraName ).CameraDisplayName );
      Output_GetCameraResolution = MyClient.GetCameraResolution( CameraName );
      fprintf( '    Resolution: %d x %d\n' , Output_GetCameraResolution.ResolutionX , Output_GetCameraResolution.ResolutionY );
      if MyClient.GetIsVideoCamera( CameraName ).IsVideoCamera
        fprintf( '    Is Video Camera: true\n' );
      else
        fprintf( '    Is Video Camera: false\n' );
      end
      
      CentroidCount = MyClient.GetCentroidCount( CameraName ).CentroidCount;
      fprintf ( '    Centroids(%d):\n', CentroidCount );

      for CentroidIndex = 1:CentroidCount
        fprintf( '      Centroid #%d:\n', CentroidIndex - 1 );

        Output_GetCentroidPosition = MyClient.GetCentroidPosition( CameraName, CentroidIndex );
        fprintf( '        Position: (%g, %g)\n', Output_GetCentroidPosition.Position( 1 ), ...
                                                 Output_GetCentroidPosition.Position( 2 ) );
        fprintf( '        Radius: (%g)\n', Output_GetCentroidPosition.Radius );
        
        Output_GetCentroidWeight = MyClient.GetCentroidWeight( CameraName, CentroidIndex );
        if( Output_GetCentroidWeight.Result.Value == Result.Success )
          fprintf( '        Weighting: %g\n', Output_GetCentroidWeight.Weight );
        end %Output_GetCentroidWeight

      end% CentroidIndex
    end% CameraIndex
  end% bReadCentroids

  waitfor(r);
end% while true  

if TransmitMulticast
  MyClient.StopTransmittingMulticast();
end  

% Disconnect and dispose
MyClient.Disconnect();

% Unload the SDK
fprintf( 'Unloading SDK...' );
Client.UnloadViconDataStreamSDK();
fprintf( 'done\n' );
