clc 
clear 
close all


%%

matrix = zeros(3);

phi = 0.9;
theta = 0.8;
psi = 0.2;

matrix(1, 1) = cos(theta)*cos(psi);
matrix(1, 2) = sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi); 
matrix(1, 3) = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
matrix(2, 1) = cos(theta)*sin(psi);
matrix(2, 2) = sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
matrix(2, 3) = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
matrix(3, 1) = -sin(theta);
matrix(3, 2) = sin(phi)*cos(theta);
matrix(3, 3) = cos(phi)*cos(theta);