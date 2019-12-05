clc 
clear 
close all

%%
% Force and Moment Notation 
% 'FX_Y' or 'MX_Y' with: 
% - X: Force nature (e.g gravitational (G), aerodynamical (A), propulsive (P) etc)
% - Y Coordinate frame (e.g Body-Fixed (B), NED (O) etc)


%% Dynamics

% Crazyflie
g = 9.81;           % gravitational acceleration in [m/s^2]
m_CF = 0.028;       % mass of the Crazyflie quadrotor in [kg]
l = 0.033;          % lever arm to the rotor in x-direction [m]
b = 0.033;          % lever arm to the rotor in y-direction [m]
I_rzz = 9.3e-8;     % Component of moment of inertia of the rotor [kg*m^2]


I_CF_1 = [16.823890  -1.224320  -0.716891    % Moment of inertia of the 
          -1.224320   16.885278  2.083147    % Crazyflie quadrotor in [kg*m^2]
          -0.716891   2.083147   29.808912] * 10e-6;       
% I_CF_1 = [16.823890     0          0       % Moment of inertia of the 
%              0      16.885278      0       % Crazyflie quadrotor in [kg*m^2]
%              0          0       29.808912] * 10e-6; 

I_CF_1_inv = inv(I_CF_1);                 % Inverted I_CF_1

% Propulsive force
k_f = 1.6e-4;       % motor constant for F_P and M_P[N/(rad/s)]
k_m = 1e-8;         % motor constant for M_P [N*m/(rad/s)]

% Aerodynanic force
kx = 10.25e-4;      % drag constant in x-direction [kg/s]
ky = 10.25e-4;      % drag constant in y-direction [kg/s]
kz = 7.553e-4;      % drag constant in z-direction [kg/s]

% Actuator dynamics 
T = 1/15;           % Actuator time constant [s]


%% Controller 

% Filter parameters 
omega_n = 50;       % in [rad/s]
zeta = 0.55;        % 

% PD gains (inner loop)
K_omega = 28.0; 
K_eta = 10.7;









