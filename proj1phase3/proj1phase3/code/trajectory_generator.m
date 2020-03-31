function s_des = trajectory_generator(t, path)

 % output desired trajectory here (given time)
s_des = zeros(13,1);
total_time = 25;
seg_num = size(path,1) - 1;
seg_time = total_time/seg_num;
T = seg_time;
segment = floor(t/seg_time)+1;
% IF time exceed total time
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
    Q = zeros(seg_num*8, seg_num*8);
    for k = 1:seg_num
        for i = 4:8
            for j = 4:8
                Q((k-1)*8+i,(k-1)*8+j) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/(j+i-7)*(seg_time)^(i+j-7);
            end
        end
    end
    %constrain matrix
    
    % waypoints position
    for i = 1:seg_num
        A(i*2-1:i*2, 8*i-7:8*i) = [1 0 0 0 0 0 0 0  ;
            1 T T^2 T^3 T^4 T^5 T^6 T^7 ;];
    end
    d = zeros(6*seg_num+2,3);
    for i = 1:seg_num
        d(i*2-1,:) = path(i,:);
        d(i*2,:) = path(i+1,:);
    end
    
    %derivative continuity constraint
    for i = 1:seg_num-1
        A((2*seg_num + 4*i-3):(2*seg_num+4*i), (8*i-7):(8*i+8))=[
            1 T T^2 T^3 T^4 T^5 T^6 T^7 -1 0 0 0 0 0 0 0 ;
            0 1 2*T 3*T^2 4*T^3 5*T^4 6*T^5 7*T^6 0 -1 0 0 0 0 0 0;
            0 0 2 6*T 12*T^2 20*T^3 30*T^4 42*T^5 0 0 -2 0 0 0 0 0;
            0 0 0 6 24*T 60*T^2 120*T^3 210*T^4 0 0 0 -6 0 0 0 0;
            ];
    end
    
    %starting and ending point
    A(6*seg_num-3:6*seg_num-1,1:8)= [0 1 0 0 0 0 0 0;
        0 0 2 0 0 0 0 0;
        0 0 0 6 0 0 0 0;];
    A(6*seg_num:6*seg_num+2, end-7:end) = [
        0 1 2*T 3*T^2 4*T^3 5*T^4 6*T^5 7*T^6;
        0 0 2 6*T 12*T^2 20*T^3 30*T^4 42*T^5;
        0 0 0 6 24*T 60*T^2 120*T^3 210*T^4;];
    
    x0 = zeros(8*seg_num,1);
    px = quadprog(Q,x0,[],[],A,d(:,1));
    py = quadprog(Q,x0,[],[],A,d(:,2));
    pz = quadprog(Q,x0,[],[],A,d(:,3));
    
    cx = px(8*segment-7:8*segment);
    cy =  py(8*segment-7:8*segment);
    cz =  pz(8*segment-7:8*segment);
    T = t - (segment-1)*T;
    s_des(1:3) = [cx';
                  cy';
                  cz';]*[1,T,T^2,T^3,T^4,T^5,T^6,T^7]';
    s_des(4:6) = [cx';
                  cy';
                  cz';]*[0,1,2*T,3*T^2,4*T^3,5*T^4,6*T^5,7*T^6]';
    des_yaw = mod(0.2 * pi * t,2 * pi);
    ypr = [des_yaw, 0.0, 0.0];
    Rot = ypr_to_R(ypr);
    q_des = R_to_quaternion(Rot);
    s_des(7:10) = q_des;
              
end
    


end


