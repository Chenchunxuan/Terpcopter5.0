function completionFlag = bhv_waypoint(stateEstimateMsg, localPositionOdomMsg, ayprCmd, completion, t )
    global timestamps
    
    tolerance_P = 0.07; % meters
    tolerance_V = 0.1; % meters
    
    error_p = [ ayprCmd.WaypointXDesiredMeters - stateEstimateMsg.East;
                ayprCmd.WaypointYDesiredMeters - stateEstimateMsg.North;
                ayprCmd.AltDesiredMeters - stateEstimateMsg.Up;];

    error_v = [ 0 - localPositionOdomMsg.Twist.Twist.Linear.X;
                0 - localPositionOdomMsg.Twist.Twist.Linear.Y;
                0 - localPositionOdomMsg.Twist.Twist.Linear.Z];  

    waypoint_pos_complete = norm(error_p) <= tolerance_P;
    waypoint_vel_complete = norm(error_v) <= tolerance_V;
    % waypointYComplete = abs(ayprCmd.WaypointYDesiredMeters - stateEstimateMsg.North) <= toleranceMeters;
    % hoverAltComplete = abs(ayprCmd.AltDesiredMeters - stateEstimateMsg.Up) <= toleranceMeters;
    % fprintf('Task: Hover at %f meters for %f seconds\n', ahs.desiredAltMeters, completion.durationSec);
    
    if waypoint_pos_complete && waypoint_vel_complete
        disp('position and velocity hold satisfied');
        current_event_time = t; % reset time for which altitude is satisfied
    else
        disp('position and velocity hold not satisfied');
        current_event_time = t;
        timestamps.behavior_satisfied_timestamp = t;
    end
    
    % require vehicle to maintain altitude within envelope for durationSec
    elapsed_satisfied_time = current_event_time - timestamps.behavior_satisfied_timestamp;
    
    fprintf('Desired Altitude: %f meters\tCurrent Altitude %f meters\nDesired Time: %f\tElapsed time: %f\nDesired Position X: %f meters\tCurrent Position X: %f meters\nDesired Position Y: %f meters\tCurrent Position Y: %f meters\n', ayprCmd.AltDesiredMeters, stateEstimateMsg.Up, completion.durationSec, elapsed_satisfied_time,ayprCmd.WaypointXDesiredMeters, stateEstimateMsg.East,ayprCmd.WaypointYDesiredMeters, stateEstimateMsg.North);
    
    if elapsed_satisfied_time >= completion.durationSec
        completionFlag = 1;
        return;
    end
    completionFlag = 0;
end