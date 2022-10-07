function y = ForwardDyn(u)

%%   motor torques
tau_1 = u(1); tau_2 = u(2); 

%% joint velocity and displacement
dtheta_1 = u(3); theta_1 = u(5); 
dtheta_2 = u(4); theta_2 = u(6); 

%%  %%%%%%%%%%%%%%%%%%%%%%%  motion profiles
% %=======================  joint 1
% A1 = 0.3; % magnitude
% f1 = 5; % frequency
% 
% %=======================  joint 2
% A2 = 0.5; % magnitude
% f2 = 2; % frequency

g = 9.801; % gravity constant

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%  link properties
%======================================  link 1
L1 = 0.5; % length [m]
c1 = L1/2; % mass center
m1 = 4.6; % mass [kg]
I1 = 1/12*m1*L1^2; % moment of inertia
%======================================  link 2
L2 = 0.3; % length [m]
c2 = L2/2; % mass center
m2 = 2.3; % mass [kg]
I2 = 1/12*m2*L2^2; % moment of inertia

%% %%%%%%%%%%%%%%%%%% dynamic simulation

    %%%%%%%%%%%%%%%%%%%%%%%%%%%  coefficients of dynamic equation
    H11 = m1*c1^2 + I1 + m2*(L1^2 + c2^2 + 2*L1*c2*cos(theta_2)) + I2;
    H22 = m2*c2^2 + I2; 
    H12 = m2*(c2^2 + L1*c2*cos(theta_2)) + I2; 
    h = m2*L1*c2*sin(theta_2); 
    G1 = m1*c1*g*cos(theta_1) + m2*g*(c2*cos(theta_1 + theta_2) + L1*cos(theta_1)); 
    G2 = m2*g*c2*cos(theta_1 + theta_2);
    
    
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  actuator torques
%     tau_1 = H11*ddtheta_1 + H12*ddtheta_2 - h*(dtheta_2)^2 ...
%         -2*h*dtheta_1*dtheta_2 + G1;
%     tau_2 = H22*ddtheta_2 + H12*ddtheta_1 + h*(dtheta_1)^2 + G2;
    
    J = [H11 H12; H12 H22]; 
    torque = [tau_1 - (- h*(dtheta_2)^2 -2*h*dtheta_1*dtheta_2 + G1);
        tau_2 - ( h*(dtheta_1)^2 + G2)];
    
    y = J\torque; 