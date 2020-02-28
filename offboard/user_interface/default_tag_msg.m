function [GLobalTagPose] = default_tag_msg()
GlobalTagPose = rosmessage('terpcopter_msgs/globaltagpos');
GlobalTagPose.Pose.Position.X = 0;
GlobalTagPose.Pose.Position.Y = 0;
GlobalTagPose.Pose.Position.Z = 0;

GLobalTagPose.Pose.Orientation.X = 0;
GlobalTagPose.Pose.Orientation.Y = 0;
GlobalTagPose.Pose.Orientation.Z = 0;
GlobalTagPose.Pose.Orientation.W = 0;
end