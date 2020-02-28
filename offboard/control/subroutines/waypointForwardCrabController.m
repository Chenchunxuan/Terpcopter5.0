function [yawStickCmd, uStickCmd, vStickCmd] = waypointForwardCrabController(curTime, yawDeg, x_d, x, y_d, y, v_x, v_y)

global prevTime x_error_body_int y_error_body_int

% gains/parameters
% Kp_u = 0.10;
% Kp_v = 0.10;
Kp = 0.20;
Kv = 0.15;
Ki = 0.03;
Int_sat = 1.0;
% Kp_w
% attitudeDeadbandMeters = 0.25;
relativeYawDeg = yawDeg - 90;  % This yaw is relative to the initial yaw when quad is turned on (90 Degrees)

% calculate attitude error in inertial frame
x_error = x_d - x;
y_error = y_d - y;

% Rotation from inertial frame to quad body frame
x_error_body = x_error*cosd(relativeYawDeg) + y_error*sind(relativeYawDeg);
y_error_body = -x_error*sind(relativeYawDeg) + y_error*cosd(relativeYawDeg);
% z_error

dt = curTime - prevTime;
y_error_body_int = y_error_body_int + y_error_body*dt;
x_error_body_int = x_error_body_int + x_error_body*dt;
if (abs(x_error_body_int)>Int_sat) x_error_body_int = sign(x_error_body_int)*Int_sat; end
if (abs(y_error_body_int)>Int_sat) y_error_body_int = sign(y_error_body_int)*Int_sat; end

% distanceErrorMeters = sqrt(x_error_body^2 + y_error_body^2); % z_error.^2

% proportional control scaled to unit vector
% uStickCmd = Kp_u*(y_error_body/distanceErrorMeters);
% vStickCmd = -Kp_v*(x_error_body/distanceErrorMeters);
uStickCmd = Kp*(y_error_body) + Kv*(-v_x) + Ki*(y_error_body_int); %pitch
vStickCmd = -(Kp*(x_error_body) + Kv*(v_y) + Ki*(x_error_body_int)); %roll
% zStickCmd

yawStickCmd = 0;

% if (abs(distanceErrorMeters) <= attitudeDeadbandMeters)
%     uStickCmd = 0;
%     vStickCmd = 0;
%     yawStickCmd = 0;
% end

prevTime = curTime;
end