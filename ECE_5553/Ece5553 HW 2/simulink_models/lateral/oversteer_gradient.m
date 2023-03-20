%Parameters
L=2.85; % Distance between the axles [m]
g=9.81; % 
l_r = 1.55;% Distance from the center of gravity of the vehicle (CG) to the rear axle
l_f = 1.3; % Distance from the center of gravity of the vehicle (CG) to the front axle
m = 2000; % Mass of the vehicle [kg]
Fz = (m*g)/4; % Vertical force is 1/4 on each tire

C_f = 3e5; % Cornering Stiffness of Front Tires
%Reduced rear cornering stiffness to make Kus < 0;
C_r = 2e5; % Cornering Stiffness of Rear Tires

%Calculate and Plot Under This Section