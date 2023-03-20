clear all
clc
close all
%% Parameters
L=2.85; % Distance between the axles [m]
g=9.81; % 
Lr = 1.55;% Distance from the center of gravity of the vehicle (CG) to the rear axle
Lf = 1.3; % Distance from the center of gravity of the vehicle (CG) to the front axle
Cf = 3e5; % Cornering Stiffness of Front Tires x2
Cr = 3e5; % Cornering Stiffness of Rear Tires x2
Cs = 1.5e5; % Cornering Stiffness
m = 2000; %Mass of the vehicle [kg]
J = 3700; %Yaw moment of Inertia
mu = 0.7; %Dry coefficient of Friction
R = 0.3; % Wheel radius
Vref = 20; % Constant vehicle velocity.
alphaf = 0.1; % Steering wheel angle rad/sec
%%

%Linear Parameters Calculation
a11 = -(Cr+Cf)/(m*Vref);
a12 = -1-((Cf*Lf-Cr*Lr)/(m*Vref^2));
a21 = (Lr*Cr-Lf*Cf)/J;
a22 = -((Cf*Lf^2)+(Cr*Lr^2))/(Vref*J);
b11 = Cf/(m*Vref);
b12 = Cr/(m*Vref); %delta_r parameter
b21 = Cf*Lf/J;
b22 = Cr*Lr/J; %delta_r parameter

e2 = 1/J; % For yaw moment term
