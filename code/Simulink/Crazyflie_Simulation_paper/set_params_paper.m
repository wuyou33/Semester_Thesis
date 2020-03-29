clc 
clear 
close all

%%
% Force and Moment Notation:
% 'FX_Y' or 'MX_Y' with: 
% - X: Force nature (e.g gravitational (G), aerodynamical (A), propulsive (P) etc)
% - Y Coordinate frame (e.g Body-Fixed (B), NED (O) etc)


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
T = 0.061658;                      % Actuator time constant [s]
omegaR_init = [1; 1; 1; 1]*200;    % Initial rotor angular velocity [rad/s]

% Matrices used for computation of the propulsive moment, G1 and G2
M11 = [-b*k_f  b*k_f  b*k_f  -b*k_f
        l*k_f  l*k_f -l*k_f  -l*k_f
        k_m   -k_m    k_m    -k_m];
    
M12 = [I_rzz  -I_rzz  I_rzz  -I_rzz 
      -I_rzz   I_rzz -I_rzz   I_rzz 
         0       0      0       0 ];
     
M2 =  [  0       0      0       0 
         0       0      0       0 
       I_rzz  -I_rzz  I_rzz  -I_rzz];


%% Controller 

% Filter parameters and coefficients
omega_n = 50;       % in [rad/s]
zeta = 0.55;        
Ts = 1/500;          % sample time in [s]

% Without frequency warping
N_0 = omega_n^2;
N_1 = 2*omega_n^2;
N_2 = omega_n^2; 

D_0 = 4/(Ts^2) + 4*zeta*omega_n/Ts + omega_n^2; 
D_1 = -8/(Ts^2) + 2*omega_n^2; 
D_2 = 4/(Ts^2) - 4*zeta*omega_n/Ts + omega_n^2; 

% With frequency warping
% N_0 = 1;
% N_1 = 2;
% N_2 = 1;
% 
% D_0 = 1/(tan(omega_n*Ts/2)^2) + 2*zeta/(tan(omega_n*Ts/2)) + 1;
% D_1 = -2/(tan(omega_n*Ts/2)^2) + 4*zeta/(tan(omega_n*Ts/2));
% D_2 = 1/(tan(omega_n*Ts/2)^2) - 2*zeta/(tan(omega_n*Ts/2)) + 1;

% Outer INDI
T_init = m_CF*0.1;      % T(t=0)!=0, prevents landing in singularity

% PD gains (inner loop) 
K_omega = 10; 
K_eta = 4;

% PD gains (outer loop)
K_d_xi = 1.5;
K_xi = 0.7;

% Workin gains 
% P = 0.7;
% D = 0.7;

% P = 1;
% D = 2;

