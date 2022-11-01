clc
clear
close all

PI = sym(pi);
syms t theta1(t) theta2(t)

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

%animation parameters
time_animation = 5;
samples = 200;

%Joint motion
theta1(t) = 3*sin(PI*t);
theta2(t) = 0.5*sin(3*PI*t+PI/4);

%Variables for link 1
l1 = 0.5;
m1 = 4.6;
I1= (1/12)*m1*l1^2;
S1(t) = [l1*cos(theta1(t)), l1*sin(theta1(t)), 0];
Sc1(t) = [(l1/2)*cos(theta1(t)); (l1/2)*sin(theta1(t)); 0];

dtheta1(t) = diff(theta1(t));
omega1(t) = dtheta1(t);
domega1(t) = diff(omega1(t));

Vc1(t) = diff(Sc1(t));
dVc1(t) = diff(Vc1(t));

%Variables for link 2
l2 = 0.5;
m2 = 2.3;
I2 = (1/12)*m2*l2^2;
Sc2(t) = [(l2/2)*cos(theta1(t)+theta2(t)); (l2/2)*sin(theta1(t)+theta2(t)); 0];
lc2 = l2/2;

dtheta2(t) = diff(theta2(t));
ddtheta2(t) = diff(dtheta2(t));
omega2(t) = dtheta1(t) + dtheta2(t);
domega2(t) = diff(omega2(t));

Vc2(t) = [-l1*dtheta1(t)*sin(theta1(t))-(dtheta1(t)+dtheta2(t))*lc2*sin(theta1(t)+theta2(t)); l1*dtheta1(t)*cos(theta1(t))+(dtheta1(t)+dtheta2(t))*lc2*cos(theta1(t)+theta2(t)); 0];
dVc2(t) = diff(Vc2(t));

g = [0; -9.8; 0];

%Forward kinematics
S1_x(t) = l1*cos(theta1(t));
S1_y(t) = l1*sin(theta1(t));
S2_x(t) = l2*cos(theta1(t)+theta2(t));
S2_y(t) = l2*sin(theta1(t)+theta2(t));

%position of EE, its velocity and acceleration
EE_x(t) = S1_x(t)+S2_x(t);
EE_y(t) = S1_y(t)+S2_y(t);

dEE_x(t) = diff(EE_x(t));
dEE_y(t) = diff(EE_y(t));

ddEE_x(t) = diff(dEE_x(t));
ddEE_y(t) = diff(dEE_y(t));

%arrays for storing the symbolic functions as discrete values
S1_dis = [];
EE_dis = [];
dEE_dis = [];
ddEE_dis = [];

k = 1;
%loop for transfering the symbolic functions into the arrays
for i = linspace(1,time_animation,samples)
    S1_dis(k,1) = S1_x(i);
    S1_dis(k,2) = S1_y(i);

    EE_dis(k,1) = EE_x(i);
    EE_dis(k,2) = EE_y(i);

    dEE_dis(k,1) = dEE_x(i)/20;
    dEE_dis(k,2) = dEE_y(i)/20;

    ddEE_dis(k,1) = ddEE_x(i)/50;
    ddEE_dis(k,2) = ddEE_y(i)/50;
    k = k+1;
end



%Torque calculation

tau2(t) = I2*domega2(t)+planarCross(Sc2(t), (m2*dVc2(t)-m2*g));

tau1(t) = I1*domega1(t)+tau2(t)+planarCross(Sc1(t), (m1.*dVc1(t)))-planarCross(Sc1(t), m1.*g)+planarCross(S1(t), m2.*dVc2(t))-planarCross(S1(t), m2.*g);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot functions for EE, velocity and acceleration

for i = 1:samples
    figure(2)
    plot([0,S1_dis(i,1),EE_dis(i,1)],[0,S1_dis(i,2),EE_dis(i,2)],'k-+');
    hold on
    plot([EE_dis(i,1),dEE_dis(i,1)],[EE_dis(i,2),dEE_dis(i,2)],"r")
    plot([EE_dis(i,1),ddEE_dis(i,1)],[EE_dis(i,2),ddEE_dis(i,2)],"b")
    hold off
    xlim([-2 2])
    ylim([-2 2])
    title("Manipulator Position Visualisation")
    legend("Position(m)", "Velocity(m/s)", "Acceleration(m/s^2)")
    xlabel("x(m)")
    ylabel("y(m)")
    grid on
  
    pause(0.03)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot functions for torques

%Plot for Tau 1
figure('Name', "Torque", 'NumberTitle', 'off')
fplot(tau1(t), [0 5], "b")
hold on

%Plot for Tau 2
fplot(tau2(t), [0 5], "r")
legend("Tau1", "Tau2")
title("Torque")
ylabel("Torque in Nm")
xlabel("Time in seconds")
hold on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot functions for thetas

%Plot for theta1
figure('Name', "Thetas", 'NumberTitle', 'off')
subplot(3, 1, 1)
fplot(theta1(t), [0 5], "b")
hold on

%Plot for theta2
fplot(theta2(t), [0 5], "r")
legend("Joint 1", "Joint 2")
title("Positions")
ylabel("Angle in radians")
xlabel("Time in seconds")
hold on

%Velocity plots
%Plot for dtheta1
subplot(3, 1, 2)
fplot(dtheta1(t), [0 5], "b")
hold on

%Plot for dtheta2
fplot(dtheta2(t), [0 5], "r")
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
hold off



