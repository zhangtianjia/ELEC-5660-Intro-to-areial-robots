% Used for HKUST ELEC 5660 

close all;
clear all;
clc;
addpath('./utils','./readonly');

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

% Environment map in 2D space 
map1 = [1.0 1.0 1.0 ; ... % start point
       1.0 2.0 1.0 ; ...  %  obstacle
       3.0 3.0 1.0 ; ...  %     .
       4.0 3.0 1.0 ; ...  %     .
       1.0 5.0 1.0 ; ... %     .
       3.0 5.0 1.0 ; ...  %     .
       2.0 7.0 1.0 ; ...  %  obstcle
       2.0 9.0 1.0 ];     % target point

map2 = [1.0 1.0 1.0 ; ...
       2.0 1.0 1.0 ; ... 
       3.0 3.0 1.0 ; ... 
       1.0 3.0 1.0 ; ... 
       2.0 5.0 1.0 ; ...
       4.0 5.0 1.0 ; ...  
       3.0 7.0 1.0 ; ...  
       4.0 9.0 1.0 ]; 
% Environment map in 2D space 
map3 = [1.0 1.0 1.0 ; ... % start point
       1.0 2.0 2.0 ; ...  %  obstacle
       3.0 3.0 3.0 ; ...  %     .
       4.0 3.0 2.0 ; ...  %     .
       1.0 5.0 4.0 ; ... %     .
       3.0 5.0 2.0 ; ...  %     .
       2.0 7.0 1.0 ; ...  %  obstcle
       4.0 9.0 5.0 ];     % target point
map4 = [1.0 1.0 1.0 ; ... % start point
       5.0 4.0 3.0 ; ...  %  obstacle
       2.0 6.0 8.0 ; ...  %     .
       4.0 3.0 7.0 ; ...  % 
       2.0 2.0 2.0 ;
       6.0 6.0 6.0 ;
       3 4 4;
       1 3 3;
       9 6 3;
       1 9 9;
       5 9 4;
       2 3 6;
       3 6 3;
       8 5 5;
       5 1 8;
       3 6 9;
       6 5 4;
       5.0 9.0 6.0 ; ... %     .
       3.0 1.0 4.0 ; ...  %     .
       4.0 8.0 9.0 ; ...  %  obstcle
       5.0 8.0 9.0 ];     % target point
test_map = map4;  % Choices: 2D: map1,map2 3D: map3, map4

% Waypoint Generator Using the A*   
%Optimal_path = path_from_A_star_2D(test_map);
Optimal_path = path_from_A_star_3D(test_map);

% Trajectory Generator Using waypoint
%trajectory_generator(t, Optimal_path, h1, test_map);

% Run Trajectory
run_trajectory_readonly(h1, h2, h3, h4, h5, h6, h7, h8, h9, Optimal_path, test_map);

