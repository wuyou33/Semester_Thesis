clc 
clear
close all

%% Parameter  

omega_n = 8;
zeta = 0.707;
Ts = 1/500;

s = tf('s');
z = tf('z', Ts);


%% Filter

Hs = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2);
Hz = c2d(Hs, Ts, 'tustin')
Hz_w = c2d(Hs, Ts, 'prewarp', omega_n)


%% Manually computed coefficients of prewrapped tustin transformation

% Without frequency prewarping
% d0 = 4/(Ts^2) + 4*zeta*omega_n/Ts + omega_n^2
% d1 = (-8/(Ts^2) + 2*omega_n^2) / d0
% d2 = (4/(Ts^2) - 4*zeta*omega_n/Ts + omega_n^2) / d0
% 
% n0 = omega_n^2 / d0
% n1 = 2*omega_n^2 / d0
% n2 = omega_n^2 / d0

% With frequency prewarping
q0 = 1/(tan(omega_n*Ts/2)^2) + 2*zeta/(tan(omega_n*Ts/2)) + 1
q1 = (-2/(tan(omega_n*Ts/2)^2) + 4*zeta/(tan(omega_n*Ts/2))) / q0
q2 = (1/(tan(omega_n*Ts/2)^2) - 2*zeta/(tan(omega_n*Ts/2)) + 1) / q0

p0 = 1 / q0
p1 = 2 / q0
p2 = 1 / q0


