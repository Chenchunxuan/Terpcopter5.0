function mission = loadMission_testhoverArtag()
mission.config.firstloop = 1;
mission.config.H-detector = 0;
mission.config.R_detector = 0;
mission.config.target_detector = 0;
mission.config.flowProbe = 0;

i = 1;
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
