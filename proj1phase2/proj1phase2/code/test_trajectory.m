% Used for HKUST ELEC 5660

close all;
clear all;
clc;
addpath('./utils','./readonly');

path1 = [0.0 0.0 1.0 ; ...
         1.0 1.0 1.0 ; ...
         -1.0 2.0 1.0 ; ...
         1.0 3.0 1.0 ; ...
         -1.0 4.0 1.0 ; ...
         1.0 5.0 1.0 ; ...
         -1.0 6.0 1.0 ; ...
         1.0 7.0 1.0 ; ...
         -1.0 8.0 1.0 ; ...
         1.0 9.0 1.0 ; ...
         0.0 10.0 1.0 ; ];
     
path2 = [0.5 0.5 1.0 ; ...
        2.0 0.5 1.0 ; ...
       2.0 2.0 1.0 ; ...
        0.5 2.0 1.0 ; ...
        0.5 0.5 1.0 ; ...
        2.0 0.5 1.0 ; ...
        2.0 2.0 1.0 ; ...
        0.5 2.0 1.0 ; ...
        0.5 0.5 1.0 ];
path3 = [0.5 0.5 0 ; ...
        2.0 0.5 1 ; ...
       2.0 2.0 2 ; ...
        0.5 2.0 1 ; ...
        0.5 0.5 0 ; ...
        2.0 0.5 -1 ; ...
        2.0 2.0 -2 ; ...
        0.5 2.0 -1 ; ...
        0.5 0.5 0 ];
path4 = [0 0 2;
         0 0.56 2.53;
         0 1.44 2.81;
         0 2.4 2.5;
         0 2.9 1.84;
         0 3.0 0.84;
         0 2.8 0.03;
         0 2.2 -0.78;
         0 1.18 -1.68;
         0 0 -2.75;
         0 -1.18 -1.68;
         0 -2.2 -0.78;
         0 -2.8 0.03;
          0 -3.0 0.84;
          0 -2.4 2.5;
          0 -1.44 2.81;
          0 -0.56 2.53;
          0 0 2;];
 global path
 global e
 path = path2;
    

    

h1 = subplot(3,3,1);
h2 = subplot(3,3,2);
h3 = subplot(3,3,3);
h4 = subplot(3,3,4);
h5 = subplot(3,3,5);
h6 = subplot(3,3,6);
h7 = subplot(3,3,7);
h8 = subplot(3,3,8);
h9 = subplot(3,3,9);
set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [100, 100, 1400, 1000]);
% set(gcf, 'WindowStyle','Modal');


% Trajectory Generator

% Run Trajectory
run_trajectory_readonly(h1, h2, h3, h4, h5, h6, h7, h8, h9,@trajectory_generator_snap);

x_rms = rms(e(1,:))
y_rms = rms(e(2,:))
z_rms = rms(e(3,:))
vx_rms = rms(e(4,:))
vy_rms = rms(e(5,:))
vz_rms = rms(e(6,:))
