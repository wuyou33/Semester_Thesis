# Semester Thesis: Design, Implementation and Evaluation of an Incremental Nonlinear Dynamic Inversion Controller for a Nano-Quadrotor

Autor: Evghenii Volodscoi

<p align="center">
	<img src="figs/indi_outer_mod_croped.jpg" width=700>
</p>

## Abstract 
Incremental Nonlinear Dynamic Inversion (INDI) is a promising control technique, widely used
for control of different types of aircraft systems. Besides providing high-performance nonlinear
control, this controller type does not require a detailed model of the controlled aircraft and is
effective against disturbances. This semester thesis describes the development of an INDI controller
to control the attitude and the position of a nano-quadrotor. It begins with the derivation
of the control algorithm. The controller is then firstly developed in the Simulink environment
and afterwards implemented on the embedded hardware of the quadrotor. Subsequently, the
implementation aspects of the INDI controller such as estimation of the control effectiveness,
measurement of the actuator time constant and estimation of the thrust mapping parameter
are discussed. Finally, the implemented controller is tested on the ability to cope with disturbance.
The final version of the implemented control algorithm is available via official open
source firmware of the Crazyflie quadrotor.

The C-code of the INDI position controller implemented in the framework of this semester thesis was merged with the official firmware of the Crazyflie Quadrotor. The corresponding pull request with detailed description of the final software structure can be found under the following link: 

https://github.com/bitcraze/crazyflie-firmware/pull/568

## Structure of the Project 

* ```code/```
    * ```actuator_dynamics/``` : Matlab scripts for estimating model of the quadcopter actuator dynamics 
    * ```binaries/```: binary files (with different controller configurations) generated for flashing onto the quadcopter hardware 
    * ```cf_logger/```: Python scripts for estimating quadcopter parameters adopted from Ewoud Smeur and Bitcraze
    * ```experiments/```: Matlab scripts for analysis of the data obtained from experiments
    * ```param_est/``` Matlab scripts for estimating quadcopter parameters such as: thrust coefficient and control effectiveness matrices
    * ```Simulink/```: Simulink model of the Crazyflie quadrotor with Simulink models of the inner and outer loop INDI controllers
    * ```transfer_functions/``` Matlab functions for stability analysis of the analytically derived transfer functions 
    * ```freq_warping.m```: Matlab script for computing Butterworth filter parameters with frequency warping
* ```crazyflie-firmware/```: official fork of the Crazyflie quadcopter firmware modified in the course of the semester thesis 
* ```figs/```: figures for README file
* ```Paper_EV.pdf```: contains final text of the semester thesis