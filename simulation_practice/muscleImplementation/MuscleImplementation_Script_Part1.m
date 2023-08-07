%%  Matlab script 
%   4 Robotics - simulation practice 
%   Muscle implementation and bioinspired control concept - Part 1

%   Martin Geiger 
%   Robin Neubauer 

clear 
clc 
close

%% Load paremeters
run init_muscle %load muscle parameters

%% Environment

Gravity=[0 0 -9.80665]; %[m/s^2]

%% Segement paramters
Head_radius=0.15; %[m]
Torso_Dimensions = [0.3 0.5 1];

Joint_radius=0.07;%[m] same radius for all visualisations of the joints
Segment_radius = 0.05; %[m] same radius for all segments

Upper_arm_length = 0.45; %[m]
Forearm_length =0.43; %[m]

