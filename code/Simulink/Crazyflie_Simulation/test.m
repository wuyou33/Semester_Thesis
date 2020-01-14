clc
clear
close all

%% Set some variables 
T = 1/15;                       % Actuator dynamics time constant [s]
Ts = 0.01;                      % Sampling time [s]

%% Filter test

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


%% Transfer functions for inner gains 

% s = tf('s');                    % Laplace variable
% z = tf('z', Ts);                % z variable
% 
% A = 1/(T*s+1);                  % Actuator dynamic in s domain              
% Az = c2d(A, Ts, 'tustin');      % Transformation from s to z domain
% 
% K_omega = 10;
% K_eta = 4;
%     
% TF_omega_ol = K_omega * Az * Ts*z/(z-1);          % Open loop tf
% TF_omega_cl = feedback(TF_omega_ol, 1);           % Closed loop tf
% 
% TF_eta_ol = K_eta * TF_omega_cl * Ts*z/(z-1);     % Open loop tf
% TF_eta_cl = feedback(TF_eta_ol, 1)               % Closed loop tf


%% Transfer functions for outer gains 

% K_d_xi = 1.5;
% K_xi = 0.7;
% 
% TF_dxi_ol = K_d_xi * TF_eta_cl * Ts*z/(z-1);     % Open loop tf
% TF_dxi_cl = feedback(TF_dxi_ol, 1);              % Closed loop tf
% 
% TF_xi_ol = K_xi * TF_dxi_cl * Ts*z/(z-1);        % Open loop tf
% TF_xi_cl = feedback(TF_xi_ol, 1);                % Closed loop tf


