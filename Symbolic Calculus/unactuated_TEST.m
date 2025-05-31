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
X01(1:5) = zeros(5,1);%[3.4907 4.132714346716140 0.2618 0.5236 5.7596].'; 
X01(2) = pi; 
X01(6:10) = zeros(5,1);
X01(1) = 0.1;

% Simulate unactuated robot to test physics
[t1,x1]=SimulateRobotTEST(X01,dt,tf,robot_params1);

% Create an animation
%clc
%Solve the Forward Kinematics
%task_pos = [p_toe1 p_knee1 p_hip pcmT p_knee2 p_toe2]; %task space configuration coordinates
toe1 = []; knee1 = []; hip = []; torsoCM = []; knee2 = []; toe2 = [];

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
d=1;
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
            %text(-1,-1.75,strcat("theta: ",num2str(rad2deg(theta1(j(i))),'%4.2f')))
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
    %drawnow;
end
text(0,1.5,"END OF SIM",'HorizontalAlignment','center')
%toc %Display total time taken to run animation (Should be ~=T_f