% Used for HKUST ELEC 5660

close all;
clear all;
clc;
addpath('./utils','./readonly');
figure(1)
h1 = subplot(3,4,1);
h2 = subplot(3,4,2);
h3 = subplot(3,4,3);
h4 = subplot(3,4,4);
h5 = subplot(3,4,6);
h6 = subplot(3,4,7);
h7 = subplot(3,4,8);
h8 = subplot(3,4,10);
h9 = subplot(3,4,11);
h10 = subplot(3,4,12);
set(gcf, 'Renderer', 'painters');

global e



% Run Trajectory  three trajectories, test one by one
run_trajectory_readonly(h1, h2, h3, h4, h5, h6, h7, h8, h9, h10, @diamond_trajectory);

x_rms = rms(e(1,:))
y_rms = rms(e(2,:))
z_rms = rms(e(3,:))
vx_rms = rms(e(4,:))
vy_rms = rms(e(5,:))
vz_rms = rms(e(6,:))
