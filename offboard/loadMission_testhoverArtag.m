function mission = loadMission_testhoverArtag()
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
% Behavior 1: Takeoff
mission.bhv{i}.name = 'bhv_artag_waypoint';
mission.bhv{i}.ayprCmd = default_aypr_msg();
mission.bhv{i}.ayprCmd.AltSwitch = 1; 
mission.bhv{i}.ayprCmd.AltDesiredMeters = 1;
% mission.bhv{i}.ayprCmd.RollSwitch = 1; 
% mission.bhv{i}.ayprCmd.RollDesiredDegrees = -20;
% mission.bhv{i}.ayprCmd.PitchSwitch = 1; 
% mission.bhv{i}.ayprCmd.PitchDesiredDegrees = 10;
mission.bhv{i}.completion.status = false;
end