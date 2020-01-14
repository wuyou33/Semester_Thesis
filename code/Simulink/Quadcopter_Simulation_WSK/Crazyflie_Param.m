% --- crazyflie parameters
g = 9.80665;

l1 = 4.65; %cm  1/2 of length of body
l2 = 4.5;


%Rotor/Propeller parameters
m = 0.00020; %propellor mass  kg
J = 1/12*m*(0.1^2+0.01^2); %moment of inertia from mass, kgm^2
b = 3.5077E-10; %motor viscous friction constant Nms


%drone parameters
m_drone = 0.03097;   % kg
Ix_drone = 1.112951*10^-5;  % kg*m^2
Iy_drone = 1.1143608*10^-5;  % kg*m^2
Iz_drone = 2.162056*10^-5;  % kg*m^2
b_drone = 1*10^-9; % kg*m^2 drone's x,y,z translational drag coefficient


