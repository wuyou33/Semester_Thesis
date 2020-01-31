clc
clear 
close all

%% 

filename = "csv_data/data_3001_1.csv";

%% Import data

[tick, gyro_roll, gyro_pitch, gyro_yaw, cmd_thrust, cmd_roll,... 
 cmd_pitch, cmd_yaw, accelz] = import_logdata(filename);