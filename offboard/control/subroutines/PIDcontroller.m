function [Yaw_rate, pitch, roll, Thrust] = PIDcontroller(curTime, yaw_d, yawDeg, x_d, x, y_d, y, v_x, v_y, z_d, z, v_z, q_m)

global prevTime x_error_int y_error_int

% gains/parameters
% Kp = -1.0;
% Kv = 0.0;
% Ki = 0.0;
Kp = 0.25;
Kv = 0.15;
Ki = 0.15;
Int_sat = 0.2;

leash_R = 0.7; %m

Kp_z = 0.8;
Kv_z = 0.3;

% Kp_yaw = 0.05;
Kp_yaw_q = 1.0;

relativeYawDeg = yawDeg - 90;  % This yaw is relative to the initial yaw when quad is turned on (90 Degrees)

% calculate attitude error in inertial frame
x_error = x_d - x;
y_error = y_d - y;

if (x_error^2 + y_error^2 > leash_R^2)
	heading = atan2(y_error,x_error);
	x_error = leash_R*cos(heading);
	y_error = leash_R*sin(heading);
end

if ( z_d > 3.0)
	z_d = 3.0;
	disp('Ceiling reached');
end
% if ( z_d < 0.4)
% 	z_d = 0.4;
% 	disp('Floor reached');
% end
z_error = z_d - z;
if (abs(z_error)>leash_R) z_error = sign(z_error)*leash_R; end

% yaw_error_deg = 180/pi*signedAngularDist( yaw_d*pi/180,relativeYawDeg*pi/180 );
% yaw_error_deg = yaw_d - relativeYawDeg;

q_ref = eul2quat([(yaw_d+90.0)*(pi/180.0) 0 0]);
q_error = quatmultiply(q_ref,quatconj(q_m));


% Rotation from inertial frame to quad body frame
x_error_body = x_error*cosd(relativeYawDeg) + y_error*sind(relativeYawDeg);
y_error_body = -x_error*sind(relativeYawDeg) + y_error*cosd(relativeYawDeg);

dt = curTime - prevTime;
y_error_int = y_error_int + y_error*dt;
x_error_int = x_error_int + x_error*dt

if (abs(x_error_int)>Int_sat) x_error_int = sign(x_error_int)*Int_sat; end
if (abs(y_error_int)>Int_sat) y_error_int = sign(y_error_int)*Int_sat; end

x_error_body_int = x_error_int*cosd(relativeYawDeg) + y_error_int*sind(relativeYawDeg);
y_error_body_int = -x_error_int*sind(relativeYawDeg) + y_error_int*cosd(relativeYawDeg)


pitch = Kp*(y_error_body) + Kv*(-v_x) + Ki*(y_error_body_int); %pitch
roll = -(Kp*(x_error_body) + Kv*(v_y) + Ki*(x_error_body_int)); %roll
Thrust = Kp_z*(z_error) + Kv_z*(-v_z);
% Yaw_rate = Kp_yaw*(yaw_error_deg);
Yaw_rate = -Kp_yaw_q*(q_error(4));

prevTime = curTime;
end