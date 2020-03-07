clc 
clear 
close all

%%
% Force and Moment Notation:
% 'FX_Y' or 'MX_Y' with: 
% - X: Force nature (e.g gravitational (G), aerodynamical (A), propulsive (P) etc)
% - Y Coordinate frame (e.g Body-Fixed (B), NED (O) etc)
% 
% Notation of other values: 
% - omegaR_X: vector of rotor angular velocities


%% Dynamics

% Crazyflie
g = 9.81;           % gravitational acceleration in [m/s^2]
m_CF = 0.028;       % mass of the Crazyflie quadrotor in [kg]
l = 0.033;          % lever arm to the rotor in x-direction [m]
b = 0.033;          % lever arm to the rotor in y-direction [m]
I_rzz = 9.3e-9;     % Component of moment of inertia of the rotor [kg*m^2]


I_CF_1 = [16.823890  -1.224320  -0.716891    % Moment of inertia of the 
          -1.224320   16.885278  2.083147    % Crazyflie quadrotor in [kg*m^2]
          -0.716891   2.083147   29.808912] * 10e-6;       
% I_CF_1 = [16.823890     0          0       % Moment of inertia of the 
%              0      16.885278      0       % Crazyflie quadrotor in [kg*m^2]
%              0          0       29.808912] * 10e-6; 

I_CF_1_inv = inv(I_CF_1);                 % Inverted I_CF_1

% Propulsive force
k_f = 1.6e-4;       % motor constant for F_P and M_P[N/(rad/s)]
k_m = 1e-6;         % motor constant for M_P [N*m/(rad/s)]

% Aerodynanic force
kx = 10.25e-4;      % drag constant in x-direction [kg/s]
ky = 10.25e-4;      % drag constant in y-direction [kg/s]
kz = 7.553e-4;      % drag constant in z-direction [kg/s]

% Actuator dynamics 
T = 1/15;           % Actuator time constant [s]


%% Controller 

% Filter parameters and coefficients
omega_n = 50;       % in [rad/s]
zeta = 0.55;        % 
Ts = 0.001;          % sample time in [s]

N_0 = omega_n^2;
N_1 = 2*omega_n^2;
N_2 = omega_n^2; 

D_0 = 4/(Ts^2) + 4*zeta*omega_n/Ts + omega_n^2; 
D_1 = -8/(Ts^2) + 2*omega_n^2; 
D_2 = 4/(Ts^2) - 4*zeta*omega_n/Ts + omega_n^2; 

% PD gains (inner loop)
K_omega = 28.0; 
K_eta = 10.7;

% PD gains (outer loop)
K_d_xi = 1.5;
K_xi = 0.7;


%% Test

%Check if manually transformed H(s) corresponds to the one computed with
%matlab

% s = tf('s');
% z = tf('z', Ts);
% 
% H = omega_n^2/(s^2 + 2*zeta*omega_n*s + omega_n^2);
% Hz = tf( (N_0/D_0*z^2 + N_1/D_0*z + N_2/D_0)/((z^2 + D_1/D_0*z + D_2/D_0)));
% Hzm = c2d(H, Ts, 'tustin')
% 
% step(H, Hz, Hzm)
% legend('H', 'Hz', 'Hzm')



