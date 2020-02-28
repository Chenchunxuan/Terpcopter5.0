function [completionFlag, GlobalTagPose] = transform(stateEstimateMsg, ARTagMsg, GlobalTagPose)
    completionFlag = 0; % 0 hardcoded only for testing.
    CamBodyTrans = [1 0 0 0;
                    0 1 0 0;
                    0 0 1 0;
                    0 0 0 1];
              
    if(isempty(ARTagMsg.Detections))
        GlobalTagPose.Pose.Position.X = 0 ;
        GlobalTagPose.Pose.Position.Y = 0 ;
        GlobalTagPose.Pose.Position.Z = 0 ;
        GlobalTagPose.Pose.Orientation.X = 0 ;
        GlobalTagPose.Pose.Orientation.Y = 0 ;
        GlobalTagPose.Pose.Orientation.Z = 0 ;
        GlobalTagPose.Pose.Orientation.W = 0 ;
        
     % class(GlobalTagPose)
    else 
        ARTagPosition = [ARTagMsg.Detections.Pose.Pose.Pose.Position.X;...
                             ARTagMsg.Detections.Pose.Pose.Pose.Position.Y;...
                             ARTagMsg.Detections.Pose.Pose.Pose.Position.Z]
            
        ARTagOrientation = [ARTagMsg.Detections.Pose.Pose.Pose.Orientation.X;...
                             ARTagMsg.Detections.Pose.Pose.Pose.Orientation.Y;...
                             ARTagMsg.Detections.Pose.Pose.Pose.Orientation.Z;...
                             ARTagMsg.Detections.Pose.Pose.Pose.Orientation.W]
                         
        BodyTagPose.POS =  CamBodyTrans*[ARTagPosition; 1];
        BodyTagPose.POSE = BodyRotCameraQuat*ARTagOrientation;
        globalposition = stateEstimateMsg.Transform*[BodyTagPose.POS;1];
        GlobalTagPose.pose.position = globalposition(1:3,1);
        GlobalTagPose.pose.orientation = stateEstimateMsg.Quat*BodyTagPose.POSE;

        class(GlobalTagPose)
        
    end    
end