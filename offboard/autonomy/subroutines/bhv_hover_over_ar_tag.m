% 1) function is expected to be called the first time only when a detection
%    is present. 
% 2) function can be called repeatedly even when there are no
%    detections. In this case the target will be the previous target
%    itself. A counter that counts number of  absent detections increments. 
% 3) if target is missing for certain number of iterations (say 100 ~10sec
%    or more) Then a reset is to be done. Upon reset 1) condition needs to
%    be satisfied again.
% 4) when detetection is present, apply control algorithm to move towards the
%    target
% 5) store the calculated vel in a cmd variable which is to be returned. 


% TODO: 
% 1) In case of no tag detection, current target = prev target + drone.
% movement in sample time. 
% 2) Change the pose rotation method. 

% TO REVIEW:
% 1) State yaw is just being added directly to tag yaw computation to give YawDesired.
% 2) Check rotation matrix for correctness.
% 

function [ completionFlag, cmd, TagInertialFrame] = bhv_hover_over_ar_tag(stateEstimateMsg, bhvTime, completion, cmd ,ARTagMsg, use_ctrl)

    %% Declaration and initialization
    persistent target errors controller first_detection_flag  absent_detections GlobatagPos; 
    
    % Global tag position
    

%     limits 
    limits.pitch_error = 20;
    limits.pitch.error_int = 5;
    limits.pitch.error_diff = 5;
    limits.roll.error = 20;
    limits.roll.error_int = 5;
    limits.roll.error_diff = 5;
    
    
    % Transform Tag from camera frame to body frame. 
    BodyTransCamera = [1 0 0 0;
                       0 1 0 0;
                       0 0 1 0;
                       0 0 0 1];
    BodyRotCameraQuat = dcm2quat(BodyTransCamera(1:3,1:3));               
    target_z = 1.5; % height for hovering.
    dt = 0.1; % time step

    % Intialize
    if isempty(first_detection_flag)
        first_detection_flag = 1;
    end

    if isempty(target)
        target.pos.curr = [0;0;0];
        target.pos.prev = [0;0;0];
        target.POSE.curr = [0;0;0];
        target.POSE.prev = [0;0;0];
    end    

    if isempty(errors)
        errors.pos.curr = [0;0;0];
        errors.pos.prev = [0;0;0];
        errors.pos.integ = [0;0;0];
        errors.pos.diff = [0;0;0];
        errors.POSE.curr = [0;0;0];
        errors.POSE.prev = [0;0;0];
        errors.POSE.integ = [0;0;0];
        errors.POSE.diff = [0;0;0];
    end    

    if isempty(controller)
        controller.pitch.Kp = [1];
        controller.pitch.Ki = [0.01];
        controller.pitch.Kd = [0];
        controller.roll.Kp = [1];
        controller.roll.Ki = [0.01];
        controller.roll.Kd = [0];
        
    end  
 
    if isempty(absent_detections)
        absent_detections = 0;
    end    
    
    %% Unpack ARtag msg
    
       
%         ARTagsec = ARTagMsg.Header.Stamp.Sec; % raw format
        if(isempty(ARTagMsg.Detections))
           ARTagPresent = 0;
           ARTagId = 0;
           ARTagPosition = [0;0;0];
           ARTagOrientation = [0;0;0;0];
           
        else    
            ARTagPresent = 1;
            ARTagId =  ARTagMsg.Detections.Id;
            
            ARTagPosition = [ARTagMsg.Detections.Pose.Pose.Pose.Position.X;...
                             ARTagMsg.Detections.Pose.Pose.Pose.Position.Y;...
                             ARTagMsg.Detections.Pose.Pose.Pose.Position.Z]
            
            ARTagOrientation = [ARTagMsg.Detections.Pose.Pose.Pose.Orientation.X;...
                                ARTagMsg.Detections.Pose.Pose.Pose.Orientation.Y;...
                                ARTagMsg.Detections.Pose.Pose.Pose.Orientation.Z;...
                                ARTagMsg.Detections.Pose.Pose.Pose.Orientation.W]
        end
      
    

    %% Cases and PID compute

    %   if detection present: 
    if ARTagPresent == 1
        if first_detection_flag == 1 
            first_detection_flag = 0;
        end
    
        % pos error calc.
        target.pos.prev = target.pos.curr; % store prev target pos 
        target.pos.curr = body_rot_camera*[ARTagPosition ; 1]; % R*pos_vec
        target.pos.curr = target.pos.curr(1:3,1); % 4x1 converted to 3x1\

        errors.pos.prev = errors.pos.curr;
        errors.pos.curr = [0;0;target_z] - target.pos.curr;
        errors.pos.integ = errors.pos.integ + errors.pos.curr*dt;
        errors.pos.diff = (errors.pos.curr - errors.pos.prev)/dt;
 
%% to review rotation   
        % POSE error calc.
        target.POSE.prev = target.POSE.curr;
        POSE_mat = body_rot_camera*[quat2rotm(ARTagOrientation'),[0;0;0];[0,0,0,1]];
        POSE_mat = POSE_mat(1:3,1:3)
        target.POSE.curr = rotm2eul(POSE_mat,"XYZ"); % Euler angles representation of tag in body frame FROM quaternion representation in camera frame. 

%%
    
        errors.POSE.prev = errors.POSE.curr;
        errors.POSE.curr = [0;0;0] - target.POSE.curr;
        errors.POSE.integ = errors.POSE.integ + errors.POSE.curr*dt;
        errors.POSE.diff = (errors.POSE.curr - errors.POSE.prev)/dt;
    
        errors = saturate(errors,limits);
        cmd = pid(errors, controller,target, cmd);

    
    elseif first_detection_flag == 1  % function is called first time 
                                                     % without any detections
    
        cmd.PitchDesiredDegrees = 0;
        cmd.RollDesiredDegrees = 0;
        cmd.YawDesiredDegrees = stateEstimateMsg.Yaw;
        
        

    else                                             % function is called without any detection
                                                     % but isn't the first call
                                                     
        absent_detections = absent_detections + 1;   % when detections not there.   
        
        % pos error calc.                                              
        target.pos.prev = target.pos.curr;
    
        errors.pos.prev = errors.pos.curr;
        errors.pos.curr = [0;0;target_z] - target.pos.curr;
        errors.pos.integ = errors.pos.integ + errors.pos.curr*dt;
        errors.pos.diff = (errors.pos.curr - errors.pos.prev)/dt;
    
        % POSE error calc. 
        target.POSE.prev = target.POSE.curr;
    
        errors.POSE.prev = errors.POSE.curr;
        errors.POSE.curr = [0;0;0] - target.POSE.curr;
        errors.POSE.integ = errors.POSE.integ + errors.POSE.curr*dt;
        errors.POSE.diff = (errors.POSE.curr - errors.pos.prev)/dt;
    
        % If no detections for 100 samples, reset errors & reset first flag    
        if absent_detections >= 100
            first_detection_flag = 1;
            errors = errors_reset(errors);
        
        end   
        
        errors = saturate(errors,limits)
        cmd = pid(errors,controller,target);
        
    end
    
    %% This is the Completion Condition for now
    % behavior completes after time elapsed
    if bhvTime >= completion.durationSec
        completionFlag = 1;
        return;
    end
    completionFlag = 0;
    %% PID Compute function
        function output = pid(errors, controller,target, output)
            % pitch 
            output.PitchDesiredDegrees = controller.pitch.Kp*errors.pos.curr(1);
            output.PitchDesiredDegrees = output.PitchDesiredDegrees + controller.pitch.Ki*errors.pos.integ(1);
            output.PitchDesiredDegrees = output.PitchDesiredDegrees + controller.pitch.Kd*errors.pos.diff(1);
            output.PitchDesiredDegrees = -1*output.PitchDesiredDegrees;
            output.PitchSwitch = 1;
       
            % roll
            output.RollDesiredDegrees = controller.roll.Kp*errors.pos.curr(2);
            output.RollDesiredDegrees = output.RollDesiredDegrees + controller.roll.Ki*errors.pos.integ(2);
            output.RollDesiredDegrees = output.RollDesiredDegrees + controller.roll.Kd*errors.pos.diff(2);
            output.RollDesiredDegrees = -1*output.RollDesiredDegrees;
            output.RollSwitch = 1;
            
            % altitude
            output.AltDesiredMeters = target_z;
            
       
            % yaw
            output.YawDesiredDegrees = target.POSE.curr(3) + stateEstimateMsg.Yaw;
       
        end
%% 

    %% PID Reset function
        function [errors] = errors_reset(errors);
            errors.pos.prev = [0;0;0];
            errors.pos.curr = [0;0;0];
            errors.pos.integ = [0;0;0];
            errors.pos.diff = [0;0;0];

            errors.POSE.prev = [0;0;0];
            errors.POSE.curr = [0;0;0];
            errors.POSE.integ = [0;0;0];
            errors.POSE.diff = [0;0;0];
        end     
    
    %% Saturate errors
        function [errors] = saturate(errors,limits)
            if abs(errors.pos.curr(1)) > abs(limits.pitch.error)
                errors.pos.curr(1) = sign(errors.pos.curr(1))*limits.pitch.error;
            end    
            if abs(errors.pos.integ(1)) > abs(limits.pitch.error_integ)
                errors.pos.integ(1) = sign(errors.pos.integ(1))*limits.pitch.error_integ;
            end
            if abs(errors.pos.diff(1)) > abs(limits.pitch.error_diff)
                errors.pos.diff(1) = sign(errors.pos.diff(1))*limits.pitch.error_diff;
            end
            if abs(errors.pos.curr(2)) > abs(limits.roll.error)
                errors.pos.curr(2) = sign(errors.pos.curr(2))*limits.roll.error;
            end
            if abs(errors.pos.integ(2)) > abs(limits.pitch.error_integ)
                errors.pos.integ(2) = sign(errors.pos.integ(2))*limits.pitch.error_integ;
            end
            if abs(errors.pos.diff(2)) > abs(limits.pitch.error_diff)
                errors.pos.diff(2) = sign(errors.pos.diff(2))*limits.pitch.error_diff;
            end
        end
    
       function [TagInertialFrame] = transform(stateEstimateMsg, ARTagPosition,ARTagOrientation)
           TagBodyFrame.POS =  BodyRotCamera*[ARTagPosition; 1];
           TagBodyFrame.POSE = BodyRotCameraQuat*ARTagOrientation;
           TagInertialFrame.POS = stateEstimateMsg.Transform*[TagBodyFrame.POS;1];
           TagInertialFrame.POSE = stateEstimateMsg.Quat*TagBodyFrame.POSE;
       end    
           
end