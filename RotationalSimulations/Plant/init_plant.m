%% init_plant - Ben Jackson    Last Edit: 7/22/24
%% Description
%  The goal of this file is to initialize the sim for testing the plant
%  system. The plant contains functions for rotating the a body in 3DoF. It
%  has NO control system. It just is used to understand 3 DoF rotation and
%  movement. This is the basic phyisics behind bodies in rotation.

addpath("Transformation Functions")

clc
clear

%% For Rotation
Euler_Initial = [0 0 0];                  % Initial Euler Angles in raidans
q_0 = Euler_To_Quaternion_ScalarFirst(Euler_Initial)'; % Initial quaternion angle
pqr_0 = [0 0 0]';                          % Initial body rates  
Moments = [0 0 0];
Inertia = [ 1 0 0 
            0 1 0
            0 0  1];

% Inertia = [ 1000 0 0 
%             0 500 0
%             0 0  1];

%% For Translation
m = 1;
g = 0;
Force_Applied = [0 0 0];
Initial_Pos =  [0 5 0];
Initial_Vel =  [0 0 0];

%% Adding certain Thrusters on Body

% Angle of Thruster
% Force of Rot Thrusters 
% Force of main Thruster


thruster_Dist_Cntr = 1;     % m
F_Rot_Thruster     = 50;    % N
F_Main_Thruster    = 50;    % N

% yaw_Euler   = [0 1 0];% Yaw   Thruster Angle
% 
%  
% pitch_Euler = [0 0 1];% Pitch Thruster Angle
% 
% 
% roll_Euler  = [1 0 0];% Roll  Thruster Angle

main_Euler = []% Fire Thruster 

% % Resulting Force and Moment 
% Moment = 
% Force  = 
% 
% 
% sim('Plant.slx');


