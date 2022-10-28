clc
clear

PI = sym(pi);
syms t

%%%%%%%%%%%%%%%%%%%%
%
% GENERAL NOTES:
% Theta corresponds to the angle of joints.
% Omega corresponds to the angular velocity of the links (note: not joints)
% EE refers to End Effector, and is either mapped in X and Y direction, og
% at the absolute value of it (e.g. velocity is given as the length of the
% velocity vector).
% Planar cross is an external function which gives the cross product of
% planar vectors.
%
%%%%%%%%%%%%%%

%Joint motion
theta1 = 3*sin(PI*t);
theta2 = 0.5*sin(3*PI*t+PI/4);

%Variables for link 1
l1 = 0.5;
m1 = 4.6;
I1= (1/12)*m1*l1^2;
Sc1 = [(l1/2)*cos(theta1); (l1/2)*sin(theta1); 0];

dtheta1 = diff(theta1);
omega1 = dtheta1;
domega1 = diff(omega1);

Vc1 = diff(Sc1);
dVc1 = diff(Vc1);

%Variables for link 2
l2 = 0.5;
m2 = 2.3;
I2 = (1/12)*m2*l2^2;
Sc2 = [(l2/2)*cos(theta1+theta2); (l2/2)*sin(theta1+theta2); 0];
lc2 = l2/2;

dtheta2 = diff(theta2);
ddtheta2 = diff(dtheta2);
omega2 = dtheta1 + dtheta2;
domega2 = diff(omega2);

Vc2 = [-l1*dtheta1*sin(theta1)-(dtheta1+dtheta2)*lc2*sin(theta1+theta2); l1*dtheta1*cos(theta1)+(dtheta1+dtheta2)*lc2*cos(theta1+theta2); 0];
dVc2 = diff(Vc2);

g = [0; -9.8; 0];

%Forward kinematics

S1_x = l1*cos(theta1);
S1_y = l1*sin(theta1);
S2_x = l2*cos(theta1+theta2);
S2_y = l2*sin(theta1+theta2);

EE_x(t) = S1_x+S2_x;
EE_y(t) = S1_y+S2_y;

dEE_x(t) = diff(EE_x);
dEE_y(t) = diff(EE_y);

ddEE_x(t) = diff(dEE_x);
ddEE_y(t) = diff(dEE_y);

time_animation = 5;
samples = 100;

EE_dis_x = [];
EE_dis_y = [];

for i = linspace(0,time_animation,samples)
    

    EE_dis_x(end+1) = EE_x(i);
    EE_dis_y(end+1) = EE_y(i);
end

for i = 1:samples
    figure(2)
    S1_x(i)
    S1_y(i)
    plot([0,S1_x(i)],[0,S1_y(i)],'k-+')
    xlim([-2 2])
    ylim([-2 2])
    grid on
    pause(0.05)
end

%Torque calculation

tau2 = I2*domega2+planarCross(Sc2, (m2*dVc2-m2*g));


tau1 = I1*domega1+tau2+planarCross(Sc1, (m1*dVc1))-planarCross(Sc1, m1*g)+planarCross(S1, m2*dVc2)-planarCross(S1, m2*g);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot functions for thetas

%Plot for theta1
figure('Name', "Thetas", 'NumberTitle', 'off')
subplot(3, 1, 1)
fplot(theta1, [0 5], "b")
hold on

%Plot for theta2
fplot(theta2, [0 5], "r")
legend("Joint 1", "Joint 2")
title("Positions")
ylabel("Angle in radians")
xlabel("Time in seconds")
hold on

%Velocity plots
%Plot for dtheta1
subplot(3, 1, 2)
fplot(dtheta1, [0 5], "b")
hold on

%Plot for dtheta2
fplot(dtheta2, [0 5], "r")
legend("Joint 1", "Joint 2")
title("Velocities")
ylabel("Angle in radians")
xlabel("Time in seconds")
hold on

%Acceleration plots
%Plot for ddtheta1
subplot(3, 1, 3)
fplot(domega1, [0 5], "b")
hold on

%Plot for ddtheta2
fplot(ddtheta2, [0 5], "r")
legend("Joint 1", "Joint 2")
title("Accelerations")
ylabel("Angle in radians")
xlabel("Time in seconds")
hold on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot functions for EE


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot functions for torques

%Plot for Tau 1
figure('Name', "Torque", 'NumberTitle', 'off')
subplot(2, 1, 1)
fplot(tau1, [0 5], "b")
hold on

%Plot for Tau 2
fplot(tau2, [0 5], "r")
legend("Tau1", "Tau2")
title("Torque")
ylabel("Torque in Nm")
xlabel("Time in seconds")
hold on
