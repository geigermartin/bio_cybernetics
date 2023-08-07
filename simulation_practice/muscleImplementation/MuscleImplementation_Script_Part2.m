%%  Matlab script 
%   4 Robotics - simulation practice 
%   Muscle implementation and bioinspired control concept - Part 2

%   The intended movement is to shoot an arrow:
%   First the model takes the arrow out of the quiver,
%   second it mounts the arrow into the bowstring,
%   and third it stretches the bowstring to then release it and shoot the
%   arrow.

%   Martin Geiger 
%   Robin Neubauer 

clear 
clc 
close

%%  Load paremeters
run init_muscle %load muscle parameters

%%  Environment
Ground = [10 4 0.02]; %[m]
Gravity = [0 0 -9.80665]; %[m/s^2]

%%  Segement paramters
Head_radius = 0.15; %[m]

Joint_radius = 0.07;%[m] same radius for all visualisations of the joints
Segment_radius = 0.05; %[m] same radius for all segments

Upper_arm_length = 0.45; %[m] same length for left and right
Forearm_length = 0.43; %[m] same length for left and right

Shoulder_length = 0.6; %[m]

Spine_length = 1; %[m]

Hip_length = 0.4; %[m]

Upper_leg_length = 0.5; %[m] same length for left and right
Lower_leg_length = 0.5; %[m] same length for left and right

%%  Segment positions
Hip_position = [0 0 1]; %[m] starting point for the multibody model

%%  Joint positions
Hip_joint_left = [0 Hip_length/2 0]; %[m]
Hip_joint_right = -Hip_joint_left; %[m]
