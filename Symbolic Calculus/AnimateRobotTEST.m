%Set robot parameters
Lt1 = 0.4; lcmt1 = 0.128; 
LT1 = 0.625; lcmT1 = 0.2;
Lf1 = 0.4; lcmf1 = 0.163;
mt1 = 3.2; mT1 = 20; mf1 = 6.8;
Jt1 = 0.93; JT1 = 2.22; Jf1 = 1.08;
g1 = 9.81;
robot_params1 = [Lt1 lcmt1 LT1 lcmT1 Lf1 lcmf1 mt1 Jt1 mT1 JT1 mf1 Jf1 g1].';
dt = 0.05;
tf = 20;
X01(1:5) = [3.4907 4.132714346716140 0.2618 0.5236 5.7596].'; 
X01(6:10) = 0.03*rand(5,1);
X01(8) = 0.1;

% Simulate unactuated robot to test physics
%[t1,x1]=SimulateRobotTEST(X01,dt,tf,robot_params1);

% Create an animation
%clc
%Solve the Forward Kinematics
%task_pos = [p_toe1 p_knee1 p_hip pcmT p_knee2 p_toe2]; %task space configuration coordinates
%loadpath = strcat('C:\Users\Philip\OneDrive\Virginia Tech\Graduate School\03-Spring 2019\ME6894 Legged Locomotion\RABBIT Midterm','\ODE Results\');
%load([loadpath 'ODE_results.mat'])
toe1 = []; knee1 = []; hip = []; torsoCM = []; knee2 = []; toe2 = [];
load ODE_results.mat
t1 = t_stance1;
x1 = q_stance1.';
for k1 = 1:length(t1)
    X1 = forwardKinematics(x1(k1,1:5).',robot_params1);
    toe1(:,k1) = X1(:,1);
    knee1(:,k1) = X1(:,2);
    hip(:,k1) = X1(:,3);
    torsoCM(:,k1) = X1(:,4);
    knee2(:,k1) = X1(:,5);
    toe2(:,k1) = X1(:,6);
end
% display every dth configuration of x
if length(t1) < 100
    d = round(length(t1)/30);
else
    d = round(length(t1)/100);
end
if length(t1)<50
    d=1;
end
%d=30; % (dt=0.005;tf=10)
%d=20; % (dt=0.005;tf=6)

%Plot Torques and GRF
figure
subplot(2,1,1)
% plot(t1,u_stance1,'LineWidth',1.4)
% legend('stance hip','swing hip','stance knee','swing knee')
plot(t1,u_stance1([1 3],:))
legend('stance hip','stance knee')
grid on
xlim([0 t1(end)]);
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(2,1,2)
plot(t1,F_stance1(1,:),t1,F_stance1(2,:))
legend('Fx','Fy')
grid on
xlabel('Time (s)')
ylabel('Force (N)')
xlim([0 t1(end)]);
%Plot Joint angles and Velocities
figure
subplot(1,3,1)
plot(t1,dq_stance1(1,:),t1,dq_stance1(2,:))
legend('dq31','dq32')
xlim([0 t1(end)]);
title('joint velocities')
grid on
subplot(1,3,2)
plot(t1,dq_stance1(3,:),t1,dq_stance1(4,:))
legend('dq41','dq42')
xlim([0 t1(end)]);
grid on
subplot(1,3,3)
plot(t1,dq_stance1(5,:))
legend('dq1')
xlim([0 t1(end)]);

grid on
figure
subplot(1,3,1)
plot(t1,q_stance1(1,:),t1,q_stance1(2,:))
legend('q31','q32')
grid on
xlim([0 t1(end)]);
subplot(1,3,2)
plot(t1,-q_stance1(3,:),t1,-q_stance1(4,:)) %flipped plots to match textbook
legend('q41','q42')
grid on
xlim([0 t1(end)]);
subplot(1,3,3)
plot(t1,q_stance1(5,:))
legend('q1')
xlim([0 t1(end)]);
title('joint angles')
grid on
% Create the Animation
figure(10)
handle = findobj(allchild(groot), 'flat', 'type', 'figure', 'number', 10); %get the handle for the animation figure
%tic; 
j=1:d:length(t1);
while ishandle(handle) %Loop the animation as long as the figure is open
    for i=1:length(j)
        hold off
        %plot(xd(1),xd(2),'kx',[x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'ko',0,0,'k^', ...
        %    [0 x1(j(i))],[0 y1(j(i))],'k',[x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'k','LineWidth',1.15)
        if ishandle(handle) %Update the animation only if the figure is open
            figure(10)
            plot([toe1(1,j(i)) knee1(1,j(i)) hip(1,j(i)) torsoCM(1,j(i))],[toe1(2,j(i)) knee1(2,j(i)) hip(2,j(i)) torsoCM(2,j(i))],'k',...
                [hip(1,j(i)) knee2(1,j(i)) toe2(1,j(i))],[hip(2,j(i)) knee2(2,j(i)) toe2(2,j(i))],'k',...
                [toe1(1,j(i)) knee1(1,j(i)) hip(1,j(i)) torsoCM(1,j(i))],[toe1(2,j(i)) knee1(2,j(i)) hip(2,j(i)) torsoCM(2,j(i))],'ko',...
                [hip(1,j(i)) knee2(1,j(i)) toe2(1,j(i))],[hip(2,j(i)) knee2(2,j(i)) toe2(2,j(i))],'ko','LineWidth',1.15);
            %text(0.75,-1.75,strcat("Time: ",num2str(j(i)*dt-dt,'%4.2f'),'s'))
            text(0.75,-1.75,strcat("Time: ",num2str(t1(j(i)),'%4.2f'),'s'))
            text(-1,-1.75,strcat("theta: ",num2str(rad2deg(theta1(j(i))),'%4.2f')))
            xlabel('x (m)')
            ylabel('y (m)')
            axis([-2 2 -2 2]);
            %axis equal
            grid
            hold on
            %pause(0.2)
            %MM(i)=getframe(gcf);
        end
    end
    drawnow;
end
text(0,1.5,"END OF SIM",'HorizontalAlignment','center')
%toc %Display total time taken to run animation (Should be ~=T_f


%% Test a single point
%close all
label = "default";
%x1 = [175; 200; 5; 5; -5].'; x1 = deg2rad(x1); %a point that is on the guard
%x1 = [3.4907 4.132714346716140 0.2618 0.5236 5.7596]; label = "q minus"; %IC for solver (x-)
%x1 = x_plus(1:5).'; label = "q plus";
%x1(5) = -0.5236%x(5) = 5.7596
%x1 = q_stance1(:,1).'; label = "X0";
%x1 = [3.55; 2.91; 0.42; 0.26; 0.055].'; %from textbook
x1 = [2.9; 3.55; 0.25; 0.42; 0.055].'; label = "textbook";

X1 = forwardKinematics(x1(1:5).',robot_params1);
toe1 = []; knee1 = []; hip = []; torsoCM = []; knee2 = []; toe2 = [];
toe1(:,1) = X1(:,1);
knee1(:,1) = X1(:,2);
hip(:,1) = X1(:,3);
torsoCM(:,1) = X1(:,4);
knee2(:,1) = X1(:,5);
toe2(:,1) = X1(:,6);
figure
plot([toe1(1) knee1(1) hip(1) torsoCM(1)],[toe1(2) knee1(2) hip(2) torsoCM(2)],'k',...
    [hip(1) knee2(1) toe2(1)],[hip(2) knee2(2) toe2(2)],'k',...
    [toe1(1) knee1(1) hip(1) torsoCM(1)],[toe1(2) knee1(2) hip(2) torsoCM(2)],'ko',...
    [hip(1) knee2(1) toe2(1)],[hip(2) knee2(2) toe2(2)],'ko','LineWidth',1.15);
xlabel('x (m)')
ylabel('y (m)')
axis([-2 2 -2 2]);
grid on
title(label)
toe2