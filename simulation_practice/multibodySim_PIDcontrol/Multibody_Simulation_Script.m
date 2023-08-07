%%  Matlab script 
%   4 Robotics - simulation practice
%   Multibody Simulation and ideal torque control concept

%   When the Matlab code is run for the first time, SIMULINK and 
%   the associated Simulink Model are automatically opened and started.
%   Also the scopes for the 4 joints are displayed automatically.

%   Martin Geiger 
%   Robin Neubauer
clear
clc
close
%% Environment

Ground= [10 4 0.02]; %[m]
Gravity=[0 0 -9.80665]; %[m/s^2]

%% Segement paramters

Joint_radius = 0.05;%[m] same radius for all visualisations of the joints
Segment_radius = 0.03; %[m] same radius for all segments

Head_radius = 0.15; %[m]

Spine_length = 0.75; %[m]

Hip_length = 0.4; %[m]

Upper_leg_length = 0.5; %[m]same length for left and right

Lower_leg_length = 0.5; %[m]same length for left and right

%% Segment positions

Hip_position = [0 0 1]; %[m] starting point for the multibody model

%% Joint positions

Hip_joint_left = [0 Hip_length/2 0]; %[m]
Hip_joint_right = -Hip_joint_left; %[m]

%% trajectory

load Gait_trajetory

%% Open and Run Simulink Model

sim('Multibody_Simulation_Simulink_M_Geiger_R_Neubauer') % Run Simulink Model

open_system('Multibody_Simulation_Simulink_M_Geiger_R_Neubauer') % Open Simulink Model

%% open scopes

scopes = find_system(gcs, 'BlockType', 'Scope');
for i=1 : length(scopes)
    set_param(scopes{i},'open','on');
end