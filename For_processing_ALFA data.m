%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% main.m - Demonstrates the usage of ALFA libraries and dataset.
% 
% For more information about the dataset, please refer to:
% http://theairlab.org/alfa-dataset
%
% For more information about this project and the publications related to 
% the dataset and this work, please refer to:
% http://theairlab.org/fault-detection-project
%
% Air Lab, Robotics Institute, Carnegie Mellon University
%
% Authors: Azarakhsh Keipour, Mohammadreza Mousaei, Sebastian Scherer
% Contact: keipour@cmu.edu
%
% Last Modified: July 16, 2020
%
% Copyright (c) 2020 Carnegie Mellon University,
% Azarakhsh Keipour <keipour@cmu.edu>
%
% For License information please see the README file in the root directory.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;
warning('off');

%% Input .mat file
% Please change the path to a sequence .mat file from the dataset
name = 'carbonZ_2018-09-11-17-27-13_2_both_ailerons_failure';

% failure_type = 0(no failure), 1(engine), 2(aileron), 3(rudder), 4(elevator)
failure_type = 2; 

filename = ['D:\My papers\202107 Fault diagnosis AI\MATLAB code\20210810 ALFA data processing\before_processed/', name, '/', name, '.mat'];

%% Load the sequence 
% Two ways to do load a sequence:
% 1 - Through the constructor:
Sequence = sequence(filename);
% 2- Loading after defining the object
% Sequence = sequence;
% Sequence.LoadSequence(filename);

%% Print brief information about the sequence
Sequence.PrintBriefInfo();

%% Load topics
% Get the start time to normalize times to start from zero
start_time = Sequence.GetStartTime();

% get topics
if failure_type == 1
    failure_status_topic = Sequence.GetTopicByName('failure_status_engines');
    failure_status_times = failure_status_topic.Data.time_recv - start_time;
    failure_status_data  = failure_status_topic.Data.data;
elseif failure_type == 2
    failure_status_topic = Sequence.GetTopicByName('failure_status_aileron');
    failure_status_times = failure_status_topic.Data.time_recv - start_time;
    failure_status_data  = failure_status_topic.Data.data;
elseif failure_type == 3
    failure_status_topic = Sequence.GetTopicByName('failure_status_rudder');
    failure_status_times = failure_status_topic.Data.time_recv - start_time;
    failure_status_data  = failure_status_topic.Data.data;
elseif failure_type == 4
    failure_status_topic = Sequence.GetTopicByName('failure_status_elevator');
    failure_status_times = failure_status_topic.Data.time_recv - start_time;
    failure_status_data  = failure_status_topic.Data.data;
end

roll_topic = Sequence.GetTopicByName('mavros_nav_info_roll');
roll_times = roll_topic.Data.time_recv - start_time;
roll_measured = roll_topic.Data.measured;
roll_commanded = roll_topic.Data.commanded;

pitch_topic = Sequence.GetTopicByName('mavros_nav_info_pitch');
pitch_times = pitch_topic.Data.time_recv - start_time;
pitch_measured = pitch_topic.Data.measured;
pitch_commanded = pitch_topic.Data.commanded;

yaw_topic = Sequence.GetTopicByName('mavros_nav_info_yaw');
yaw_times = yaw_topic.Data.time_recv - start_time;
yaw_measured = yaw_topic.Data.measured;
yaw_commanded = yaw_topic.Data.commanded;

velocity_topic = Sequence.GetTopicByName('mavros_nav_info_velocity');
velocity_times = velocity_topic.Data.time_recv - start_time;
velocity_measured_x = velocity_topic.Data.meas_x;
velocity_measured_y = velocity_topic.Data.meas_y;
velocity_measured_z = velocity_topic.Data.meas_z;
velocity_commanded_x = velocity_topic.Data.des_x;
velocity_commanded_y = velocity_topic.Data.des_y;
velocity_commanded_z = velocity_topic.Data.des_z;

local_position_velocity_topic = Sequence.GetTopicByName('mavros_local_position_velocity');
local_position_velocity_times = local_position_velocity_topic.Data.time_recv - start_time;
local_position_velocity_twist =  local_position_velocity_topic.Data.twist;
twist_length = length(local_position_velocity_twist);
angular_velocity_measured_x = zeros(twist_length, 1);
angular_velocity_measured_y = zeros(twist_length, 1);
angular_velocity_measured_z = zeros(twist_length, 1);
for i = 1:twist_length
    angular_velocity_measured_x(i) = local_position_velocity_twist(i).angular.x;
    angular_velocity_measured_y(i) = local_position_velocity_twist(i).angular.y;
    angular_velocity_measured_z(i) = local_position_velocity_twist(i).angular.z;
end

local_position_pose_topic = Sequence.GetTopicByName('mavros_local_position_pose');
local_position_pose_times = local_position_pose_topic.Data.time_recv - start_time;
local_position_pose_pose =  local_position_pose_topic.Data.pose;
pose_length = length(local_position_pose_pose);
position_measured_x = zeros(pose_length, 1);
position_measured_y = zeros(pose_length, 1);
position_measured_z = zeros(pose_length, 1);
for i = 1:pose_length
    position_measured_x(i) = local_position_pose_pose(i).position.x;
    position_measured_y(i) = local_position_pose_pose(i).position.y;
    position_measured_z(i) = local_position_pose_pose(i).position.z;
end

% % Plot the data
% figure;
% plot(roll_times, roll_topic.Data.measured);
% hold on; grid on;
% plot(roll_times, roll_topic.Data.commanded);
% hold on;
% plot(failure_status_times, failure_status_data);
% legend('measured', 'commanded', 'failure');
% hold off

%% Data structure
% 1. Time 4Hz
end_time = roll_times(end);
times = [0:0.25:end_time];
times = times(2:end-1);
times_length = length(times);

% 2. Failure status
% engine = 0 (normal), 1 (failure)
% aileron = 0 (normal), 1 (right side failure), 2 (left side failure), 3
% (both side failure)
% rudder = 0 (normal), 1 (stuck in zero position), 2(stuck all the way to the
% left), 3 (stuck all the way to the right)
% elevator = 0 (normal), 1 (stuck in zero position), 2 (stuck all the way
% down)

failure_status = zeros(4, times_length);

% 3. States and inputs
% measured position in NED (3), 
% measured velocity (3), 
% measured attitude in roll/pitch/yaw (3),
% measured angular velocity(3),
% commanded velocity (3),
% commanded attitude in roll/pitch/yaw (3).

states_inputs = zeros(18, times_length);

%% Data processing

% labels
for i = 1:times_length
    if failure_type == 1
        index_list = find(failure_status_times < times(i)); 
        if(isempty(index_list))
            continue
        else
            index = index_list(end);
            failure_status(1, i) = failure_status_data(index);
        end
    elseif failure_type == 2
        index_list = find(failure_status_times < times(i)); 
        if(isempty(index_list))
            continue
        else
            index = index_list(end);
            failure_status(2, i) = failure_status_data(index);
        end
    elseif failure_type == 3
        index_list = find(failure_status_times < times(i)); 
        if(isempty(index_list))
            continue
        else
            index = index_list(end);
            failure_status(3, i) = failure_status_data(index);
        end
    elseif failure_type == 4
        index_list = find(failure_status_times < times(i)); 
        if(isempty(index_list))
            continue
        else
            index = index_list(end);
            failure_status(4, i) = failure_status_data(index);
        end
    end 
end

% states
for i = 1:times_length
    % measured position in NED (3), 
    index_list = find(local_position_pose_times < times(i)); 
    if(isempty(index_list))
        continue
    else
        index = index_list(end);
        states_inputs(1, i) = position_measured_x(index);
        states_inputs(2, i) = position_measured_y(index);
        states_inputs(3, i) = position_measured_z(index);
    end
    
    % measured velocity(3), commanded velocity (3)
    index_list = find(velocity_times < times(i)); 
    if(isempty(index_list))
        continue
    else
        index = index_list(end);
        states_inputs(4, i) = velocity_measured_x(index);
        states_inputs(5, i) = velocity_measured_y(index);
        states_inputs(6, i) = velocity_measured_z(index);
        states_inputs(13, i) = velocity_commanded_x(index);
        states_inputs(14, i) = velocity_commanded_y(index);
        states_inputs(15, i) = velocity_commanded_z(index);
    end
    
    % measured attitude in roll/pitch/yaw (3),
    % commanded attitude in roll/pitch/yaw (3),
    index_list = find(roll_times < times(i)); 
    if(isempty(index_list))
        continue
    else
        index = index_list(end);
        states_inputs(7, i) = roll_measured(index);
        states_inputs(8, i) = pitch_measured(index);
        states_inputs(9, i) = yaw_measured(index);
        states_inputs(16, i) = roll_commanded(index);
        states_inputs(17, i) = pitch_commanded(index);
        states_inputs(18, i) = yaw_commanded(index);
    end
    
    % measured angular velocity(3),
    index_list = find(local_position_velocity_times < times(i)); 
    if(isempty(index_list))
        continue
    else
        index = index_list(end);
        states_inputs(10, i) = angular_velocity_measured_x(index);
        states_inputs(11, i) = angular_velocity_measured_y(index);
        states_inputs(12, i) = angular_velocity_measured_z(index);
    end
end

save_filename = ['D:\My papers\202107 Fault diagnosis AI\MATLAB code\20210810 ALFA data processing\data\', name, '.mat'];
save(save_filename, 'failure_status', 'states_inputs');

