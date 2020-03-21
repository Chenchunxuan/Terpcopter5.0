% AMAV 2020 Vicon Motion Capture Plotting
%
% Author: Zachary Lacey
% Email: zlacey@umd.edu

clear all 
close all
clc

%
[file3, path3] = uigetfile('*.log');

%% Vicon Motion Capture
file3
filepath3 = [path3 file3];
data3 = csvread(filepath3);

% parse out
SubjectIndex = data3(:,1);

TimeVicon = data3(:,2);

PositionXVicon = data3(:,3)/1000;
PositionYVicon = data3(:,4)/1000;
PositionZVicon = data3(:,5)/1000;

OrientationPhiVicon = data3(:,6);
OrientationThetaVicon = data3(:,7);
OrientationPsiVicon = data3(:,8);


%% Plotting Postion

figure(1)
hold on
plot3(PositionXVicon,PositionYVicon,PositionZVicon);
title('3D Plot of Vicon Motion Capture vs Intel Realsense T265');
xlabel('Position X (m)');
ylabel('Position Y (m)');
zlabel('Position Z (m)');
legend('Vicon (Ground Truth)', 'Realsense T265');
set(gca, 'FontSize', 16);
axis equal
grid on 
hold off
