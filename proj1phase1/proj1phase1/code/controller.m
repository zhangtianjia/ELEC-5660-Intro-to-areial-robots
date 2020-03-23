function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;
kp_pos = [3 3 3];
kd_pos = [7 7 7];
kp_atd = [1000 1000 500];
kd_atd = [100 100 20];
x = s(1);
y = s(2);
z = s(3);
xdot = s(4);
ydot = s(5);
zdot = s(6);
qW = s(7);
qX = s(8);
qY = s(9);
qZ = s(10);
p = s(11);
q = s(12);
r = s(13);
x_des = s_des(1);
y_des = s_des(2);
z_des = s_des(3);
xdot_des = s_des(4);
ydot_des = s_des(5);
zdot_des = s_des(6);
qW_des = s_des(7);
qX_des = s_des(8);
qY_des = s_des(9);
qZ_des = s_des(10);
p_des = s_des(11);
q_des = s_des(12);
r_des = s_des(13);
Rot = QuatToRot([qW,qX,qY,qZ]');
[phi,theta,yawangle] = RotToRPY_ZXY(Rot);
Rot_des = QuatToRot(s_des(7:10));
[phi_des,theta_des,yawangle_des] = RotToRPY_ZXY(Rot_des);
pos_accel_des = [g*(theta_des * cos(yawangle_des)+phi_des*sin(yawangle_des))...
                g*(theta_des * sin(yawangle_des)-phi_des*cos(yawangle_des))...
                0];
pos = [x, y, z]';
pos_des = [x_des, y_des, z_des]';
pos_dot = [xdot, ydot, zdot]';
pos_dot_des = [xdot_des, ydot_des, zdot_des]';
% postion control
pos_accel_c = pos_accel_des + kd_pos.*(pos_dot_des - pos_dot) + kp_pos.*(pos_des -pos);
u1 = m*g +m*pos_accel_c(3);

% attitude control
phi_c = 1/g*(pos_accel_c(1)*sin(yawangle) - pos_accel_c(2)*cos(yawangle));
theta_c = 1/g*(pos_accel_c(1)*cos(yawangle) +pos_accel_c(2)*sin(yawangle));
w = s(11:13);
yaw_angle_dif = yawangle_des-yawangle;
if yaw_angle_dif >= pi
    yaw_angle_dif=yaw_angle_dif-2*pi;
elseif yaw_angle_dif <= -pi
    yaw_angle_dif=yaw_angle_dif+2*pi;
end
atd_accel_c = [kp_atd(1)*(phi_c-phi)+kd_atd(1)*(0-w(1));
                 kp_atd(2)*(theta_c-theta)+kd_atd(2)*(0-w(2));
                 kp_atd(3)*(yaw_angle_dif)+kd_atd(3)*(0-w(3))];
u2 = I*atd_accel_c + cross(w, I*w);

F = u1; M = u2; % You should calculate the output F and M


end
