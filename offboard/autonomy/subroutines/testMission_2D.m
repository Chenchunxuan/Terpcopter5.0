function mission = testMission_2D()
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

i = 1;
j = 1;
% Behavior 1: Takeoff
mission.bhv{i}{j}.name = 'bhv_takeoff';
mission.bhv{i}{j}.ayprCmd = default_aypr_msg();
mission.bhv{i}{j}.ayprCmd.AltSwitch = 1; 
mission.bhv{i}{j}.ayprCmd.AltDesiredMeters = 1;
% mission.bhv{i}.ayprCmd.RollSwitch = 1; 
% mission.bhv{i}.ayprCmd.RollDesiredDegrees = -20;
% mission.bhv{i}.ayprCmd.PitchSwitch = 1; 
% mission.bhv{i}.ayprCmd.PitchDesiredDegrees = 10;
mission.bhv{i}{j}.completion.status = false;

j = j + 1;
% Behavior 2: Hover
mission.bhv{i}{j}.name = 'bhv_hover';
mission.bhv{i}{j}.ayprCmd = default_aypr_msg();
mission.bhv{i}{j}.ayprCmd.AltSwitch = 1; 
mission.bhv{i}{j}.ayprCmd.AltDesiredMeters = 1; 
mission.bhv{i}{j}.completion.durationSec = 15; % 10 seconds
mission.bhv{i}{j}.completion.status = false;     % completion flag


j = j + 1;
% Behavior 3: Land
mission.bhv{i}{j}.name = 'bhv_land';
mission.bhv{i}{j}.ayprCmd = default_aypr_msg();
mission.bhv{i}{j}.ayprCmd.AltSwitch = 1; 
mission.bhv{i}{j}.ayprCmd.AltDesiredMeters = 0.0; 
mission.bhv{i}{j}.completion.durationSec = 10; % make this very long so vehicle hovers above ground before manual takeover
mission.bhv{i}{j}.completion.status = false;     % completion flag

end