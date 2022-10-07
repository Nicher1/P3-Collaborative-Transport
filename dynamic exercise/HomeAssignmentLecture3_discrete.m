clear all
clc

PI = sym(pi);
syms t

%Variables for link 1
l1 = 0.5;
m1 = 4.6;
I1= (1/12)*m1*l1^2; 

%Variables for link 2
l2 = 0.5;
m2 = 2.3;
I2 = (1/12)*m2*l2^2;
    
%Joint motion
theta1 = 3*sin(PI*t);
theta2 = 0.5*sin(3*PI*t+PI/4);

dtheta1 = diff(theta1);
dtheta2 = diff(theta2);

ddtheta1 = diff(dtheta1);
ddtheta2 = diff(dtheta2);

g = 9.8;

%Forward kinematics

S1 = [l1*cos(theta1); l1*sin(theta1)];
S2 = [l2*cos(theta1+theta2); l2*sin(theta1+theta2)];

dEE = diff(S1+S2);
ddEE = diff(dEE);

samples = 100;
time = linspace(0,3,samples);
i = 0;

for p = linspace(0,1,samples)
    i = i+1; time(i) = p;
    t = p;

    %discrete joint values
    dis_theta1(i) = subs(theta1);
    dis_theta2(i) = subs(theta2);

    dis_dtheta1(i) =subs(dtheta1);
    dis_dtheta2(i) =subs(dtheta2);

    dis_ddtheta1(i) =subs(ddtheta1);
    dis_ddtheta2(i) =subs(ddtheta2);

    %discrete EE position
    S1_x(i) = l1*cos(dis_theta1(i));
    S1_y(i) = l1*sin(dis_theta1(i));

    S2_x(i) = l2*cos(dis_theta1(i) + dis_theta2(i));
    S2_y(i) = l2*sin(dis_theta1(i) + dis_theta2(i));

    %dis_S1(2) = ;
end

% Plotting angular movement
figure(1)
subplot(3,1,1)
hold on 
plot(time, dis_theta1,'b')
plot(time, dis_theta2,'r')
hold off
grid on
legend('joint 1','joint 2')
xlabel('time[s]')
ylabel('angular placement[rad]')
subplot(3,1,2)
hold on 
plot(time, dis_dtheta1,'b')
plot(time, dis_dtheta2,'r')
hold off
grid on
xlabel('time[s]')
ylabel('angular velocity[rad/s]')
subplot(3,1,3)
hold on 
plot(time, dis_ddtheta1,'b')
plot(time, dis_ddtheta2,'r')
hold off
grid on
xlabel('time[s]')
ylabel('angular acceleration[rad/s^2]')

%Plotting EE placement
figure(2)

for j = 1 : samples   
    plot([0,S1_x(j), S1_x(j)+S2_x(j)],[0,S1_y(j), S1_y(j)+S2_y(j)],'k-+')
    xlim([-1 1])
    ylim([-1 1])
    grid on
    % 
    pause(0.05)
end
