function s_des = trajectory_generator_polynomial(t)
global path
s_des = zeros(13,1);
total_time = 25;
seg_num = size(path,1) - 1;
seg_time = total_time/seg_num;
segment = floor(t/seg_time)+1;
if segment+1>size(path,1)
   x_des = path(size(path,1),1) ;
   vx_des =0;
   y_des = path(size(path,1),2);
   vy_des = 0;
   z_des =path(size(path,1),3);
   vz_des = 0;
s_des(1:6) = [x_des y_des z_des vx_des vy_des vz_des]';

des_yaw = mod(0.2 * pi * t,2 * pi);
ypr = [des_yaw, 0.0, 0.0];
Rot = ypr_to_R(ypr);
q_des = R_to_quaternion(Rot);
s_des(7:10) = q_des;
else
    
a = path(segment,:)';
b = path(segment+1,:)';
v = (b - a)/seg_time;
T = seg_time;
A=[0 0 0 0 0 1;
   T^5 T^4 T^3 T^2 T 1 ;
   0 0 0 0 1 0;
   5*T^4 4*T^3 3*T^2 2*T 1 0 ;
   0 0 0 2 0 0;
   20*T^3 12*T^2 6*T 2 0 0;
   ];
cx = inv(A)*[a(1) b(1) v(1) v(1) 0 0]';
cy = inv(A)*[a(2) b(2) v(2) v(2) 0 0]';
cz = inv(A)*[a(3) b(3) v(3) v(3) 0 0]';
t_seg = t - (segment-1)*T;
   x_des = [t_seg^5 t_seg^4 t_seg^3 t_seg^2 t_seg 1]*cx;
   vx_des =[5*t_seg^4 4*t_seg^3 3*t_seg^2 2*t_seg 1 0]*cx;
   y_des =[t_seg^5 t_seg^4 t_seg^3 t_seg^2 t_seg 1]*cy;
   vy_des = [5*t_seg^4 4*t_seg^3 3*t_seg^2 2*t_seg 1 0]*cy;
   z_des =[t_seg^5 t_seg^4 t_seg^3 t_seg^2 t_seg 1]*cz;
   vz_des = [5*t_seg^4 4*t_seg^3 3*t_seg^2 2*t_seg 1 0]*cz;
s_des(1:6) = [x_des y_des z_des vx_des vy_des vz_des]';

des_yaw = mod(0.2 * pi * t,2 * pi);
ypr = [des_yaw, 0.0, 0.0];
Rot = ypr_to_R(ypr);
q_des = R_to_quaternion(Rot);
s_des(7:10) = q_des;
end    
end




