%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%ECE 5553 - Autonomy in Vehicles
%%HW 4 - Path Following Linear Model
%%Spring 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Simulation file
clear all
close all

[L, g, Lr, Lf, Cf, Cr, m, J, mu, R, Cs, V, ls, a11, a12, a21, a22, b11, b12, b21, b22, e2, A_matrix, B_matrix, C_matrix, D_matrix] = getModel();

%Run the Double lane change manoeuver
Double_lane_change_bezier_func_mcode_V2

sim('Path_following_Model_w_dbl_lane_change');

figure(1);
plot(X_ref,Y_ref,'r--','Linewidth',2); hold on;grid on;
plot(X_actual,Y_actual,'b','Linewidth',2); hold on;grid on;

legend('Reference','Actual')
xlabel('X Position [m]')
ylabel('Y Position [m]')
title('X-Y path')


function [L, g, Lr, Lf, Cf, Cr, m, J, mu, R, Cs, V, ls, a11, a12, a21, a22, b11, b12, b21, b22, e2, A_matrix, B_matrix, C_matrix, D_matrix] = getModel()
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%ECE 5553 - Autonomy in Vehicles
    %%HW 4 - Path Following Linear Model
    %%Spring 2019
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%Vehicle Parameters
    L=2.85; % Distance between the axles [m]
    g=9.81; % 
    Lr = 1.55;% Distance from the center of gravity of the vehicle (CG) to the rear axle
    Lf = 1.3; % Distance from the center of gravity of the vehicle (CG) to the front axle
    Cf = 3e5; % Cornering Stiffness of Front Tires
    Cr = 3e5; % Cornering Stiffness of Rear Tires
    m = 2000; %Mass of the vehicle [kg]
    J = 3700; %Yaw moment of Inertia
    mu = 0.7; %Dry coefficient of Friction
    R = 0.3;
    Cs=1.5e5;
    
    %%Operating condition
    V = 5; %Longitudinal Velocity [m/s]
    ls = 2; %Preview Distance [m]
    
    %Linear Parameters Calculation
    a11 = -(Cr+Cf)/(m*V);
    a12 = -1-((Cf*Lf-Cr*Lr)/(m*V^2));
    a21 = (Lr*Cr-Lf*Cf)/J;
    a22 = -((Cf*Lf^2)+(Cr*Lr^2))/(V*J);
    b11 = Cf/(m*V); %Only Front wheel steering
    b12 = 0; %delta_r parameter
    b21 = Cf*Lf/J; %Only Front wheel steering
    b22 = 0; %delta_r parameter
    e2 = 1/J; % For yaw moment term - Not used in Path following model currently
    
    %%%%State Space Representation
    A_matrix = [a11 a12 0 0; a21 a22 0 0; 0 1 0 0; V ls V 0];
    B_matrix = [b11 0; b21 0; 0 -V; 0 -ls*V];
    C_matrix = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]; %Output is beta, r and ey
    D_matrix = [0 0; 0 0; 0 0; 0 0];
end 

function [Xp,Yp] = bezier_curve(p1)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%ECE 5553 - Autonomy in Vehicles
    %%HW 4 - Path Following Linear Model
    %%Spring 2019
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%Bezier curve for higher order polynomials%
    % https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-837-computer-graphics-fall-2012/lecture-notes/MIT6_837F12_Lec01.pdf
    % Refer to Slide 62
    
    %   Detailed explanation goes here
    p = p1;
    
    n = length(p); %number of points
    n1=n-1;
    
    for i=0:1:n1
    sigma(i+1)=factorial(n1)/(factorial(i)*factorial(n1-i));  % for calculating (x!/(y!(x-y)!)) values 
    end
    l=[];
    UB=[];
    for u=0:0.002:1
        for d=1:n
        UB(d)=sigma(d)*((1-u)^(n-d))*(u^(d-1));
        end
    l=cat(1,l,UB);                                       
    end
    P=l*p;
    
    Xp = [P(:,1)]; %X_reference Points
    Yp = [P(:,2)]; %Y_reference Points
end
