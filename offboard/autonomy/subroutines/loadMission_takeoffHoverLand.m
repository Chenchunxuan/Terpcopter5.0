function mission = loadMission_takeoffHoverLand()
mission.config.firstLoop = 1;

mission.config.H_detector = 0;
mission.config.R_detector = 0;
mission.config.target_detector = 0;
mission.config.flowProbe = 0;

% for reference:
%
% ayprCmdMsg = rosmessage(ayprCmdPublisher);
% ayprCmdMsg.AltDesiredMeters = 0;
% ayprCmdMsg.YawDesiredDeg = 0;
% ayprCmdMsg.PitchDesiredDeg = 0;
% ayprCmdMsg.RollDesiredDeg = 0;
% ayprCmdMsg.AltSwitch = 0;
% ayprCmdMsg.YawSwitch = 0;
% ayprCmdMsg.PitchSwitch = 0;
% ayprCmdMsg.RollSwitch = 0;

% i = 1;
% Behavior 1: Takeoff
% mission.bhv{i}.name = 'bhv_takeoff';
% mission.bhv{i}.ayprCmd = default_aypr_msg();
% mission.bhv{i}.ayprCmd.AltSwitch = 1; 
% mission.bhv{i}.ayprCmd.AltDesiredMeters = 1.0;
% mission.bhv{i}.ayprCmd.RollSwitch = 1; 
% mission.bhv{i}.ayprCmd.RollDesiredDegrees = -20;
% mission.bhv{i}.ayprCmd.PitchSwitch = 1; 
% mission.bhv{i}.ayprCmd.PitchDesiredDegrees = 10;
% mission.bhv{i}.completion.status = false;

% i = i + 1;
% % Behavior 2: Hover
% mission.bhv{i}.name = 'bhv_hover';
% mission.bhv{i}.ayprCmd = default_aypr_msg();
% mission.bhv{i}.ayprCmd.AltSwitch = 1; 
% mission.bhv{i}.ayprCmd.AltDesiredMeters = 1.5; 
% mission.bhv{i}.completion.durationSec = 5; % 10 seconds
% mission.bhv{i}.completion.status = false;     % completion flag

i = 1;
% Behavior 2: Waypoint
mission.bhv{i}.name = 'bhv_waypoint';
mission.bhv{i}.ayprCmd = default_aypr_msg();
% mission.bhv{i}.ayprCmd.AltSwitch = 1; 
mission.bhv{i}.ayprCmd.AltDesiredMeters = 0.75; 
mission.bhv{i}.ayprCmd.WaypointSwitch = 1;
mission.bhv{i}.ayprCmd.WaypointXDesiredMeters = 0.0;
mission.bhv{i}.ayprCmd.WaypointYDesiredMeters = 0.0;
mission.bhv{i}.ayprCmd.YawDesiredDegrees = 0.0;
mission.bhv{i}.completion.durationSec = 0.1; % 10 seconds
mission.bhv{i}.completion.status = false;     % completion flag

i = i + 1;
% Behavior 3: Waypoint
mission.bhv{i}.name = 'bhv_waypoint';
mission.bhv{i}.ayprCmd = default_aypr_msg();
% mission.bhv{i}.ayprCmd.AltSwitch = 1; 
mission.bhv{i}.ayprCmd.AltDesiredMeters = 1.75; 
mission.bhv{i}.ayprCmd.WaypointSwitch = 1;
mission.bhv{i}.ayprCmd.WaypointXDesiredMeters = 1.0;
mission.bhv{i}.ayprCmd.WaypointYDesiredMeters = 0.0;
mission.bhv{i}.ayprCmd.YawDesiredDegrees = -90.0;
mission.bhv{i}.completion.durationSec = 0.1; % 10 seconds
mission.bhv{i}.completion.status = false;     % comletion flag

i = i + 1;
% Behavior 3: Waypoint
mission.bhv{i}.name = 'bhv_waypoint';
mission.bhv{i}.ayprCmd = default_aypr_msg();
% mission.bhv{i}.ayprCmd.AltSwitch = 1; 
mission.bhv{i}.ayprCmd.AltDesiredMeters = 0.75; 
mission.bhv{i}.ayprCmd.WaypointSwitch = 1;
mission.bhv{i}.ayprCmd.WaypointXDesiredMeters = 1.0;
mission.bhv{i}.ayprCmd.WaypointYDesiredMeters = 1.0;
mission.bhv{i}.ayprCmd.YawDesiredDegrees = 0.0;
mission.bhv{i}.completion.durationSec = 0.1; % 10 seconds
mission.bhv{i}.completion.status = false;     % comletion flag

i = i + 1;
% Behavior 3: Waypoint
mission.bhv{i}.name = 'bhv_waypoint';
mission.bhv{i}.ayprCmd = default_aypr_msg();
% mission.bhv{i}.ayprCmd.AltSwitch = 1; 
mission.bhv{i}.ayprCmd.AltDesiredMeters = 1.75; 
mission.bhv{i}.ayprCmd.WaypointSwitch = 1;
mission.bhv{i}.ayprCmd.WaypointXDesiredMeters = 0.0;
mission.bhv{i}.ayprCmd.WaypointYDesiredMeters = 1.0;
mission.bhv{i}.ayprCmd.YawDesiredDegrees = 90.0;
mission.bhv{i}.completion.durationSec = 0.1; % 10 seconds
mission.bhv{i}.completion.status = false;     % comletion flag

i = i + 1;
% Behavior 3: Waypoint
mission.bhv{i}.name = 'bhv_waypoint';
mission.bhv{i}.ayprCmd = default_aypr_msg();
% mission.bhv{i}.ayprCmd.AltSwitch = 1; 
mission.bhv{i}.ayprCmd.AltDesiredMeters = 0.75; 
mission.bhv{i}.ayprCmd.WaypointSwitch = 1;
mission.bhv{i}.ayprCmd.WaypointXDesiredMeters = 0.0;
mission.bhv{i}.ayprCmd.WaypointYDesiredMeters = 0.0;
mission.bhv{i}.ayprCmd.YawDesiredDegrees = 0.0;
mission.bhv{i}.completion.durationSec = 0.1; % 10 seconds
mission.bhv{i}.completion.status = false;     % comletion flag

i = i + 1;
% Behavior 3: Waypoint
mission.bhv{i}.name = 'bhv_waypoint';
mission.bhv{i}.ayprCmd = default_aypr_msg();
% mission.bhv{i}.ayprCmd.AltSwitch = 1; 
mission.bhv{i}.ayprCmd.AltDesiredMeters = 0.70; 
mission.bhv{i}.ayprCmd.WaypointSwitch = 1;
mission.bhv{i}.ayprCmd.WaypointXDesiredMeters = 0.0;
mission.bhv{i}.ayprCmd.WaypointYDesiredMeters = 0.0;
mission.bhv{i}.ayprCmd.YawDesiredDegrees = 0.0;
mission.bhv{i}.completion.durationSec = 0.1; % 10 seconds
mission.bhv{i}.completion.status = false;     % comletion flag

% i = i + 1;
% % Behavior 3: Waypoint
% mission.bhv{i}.name = 'bhv_waypoint';
% mission.bhv{i}.ayprCmd = default_aypr_msg();
% % mission.bhv{i}.ayprCmd.AltSwitch = 1; 
% mission.bhv{i}.ayprCmd.AltDesiredMeters = 1.0; 
% mission.bhv{i}.ayprCmd.WaypointSwitch = 1;
% mission.bhv{i}.ayprCmd.WaypointXDesiredMeters = 0.0;
% mission.bhv{i}.ayprCmd.WaypointYDesiredMeters = 0.0;
% mission.bhv{i}.completion.durationSec = 0.1; % 10 seconds
% mission.bhv{i}.completion.status = false;     % comletion flag

% i = i + 1;
% % Behavior 4: Land
% mission.bhv{i}.name = 'bhv_land';
% mission.bhv{i}.ayprCmd = default_aypr_msg();
% mission.bhv{i}.ayprCmd.AltSwitch = 1; 
% mission.bhv{i}.ayprCmd.AltDesiredMeters = 0.0; 
% mission.bhv{i}.completion.durationSec = 20; % make this very long so vehicle hovers above ground before manual takeover
% mission.bhv{i}.completion.status = false;     % completion flag

end