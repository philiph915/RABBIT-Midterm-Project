%% ME6984 Feedback Control of Dynamic Legged Locomotion 
% Midterm Project Part 3
% By Philip Hancock
% 4/10/2019
%
% Simulate the robot walking for 20 steps with an initial condition
% slightly off from the optimal gait
%% Specifty robot parameters
% Values taken from journal paper
mT = 20; mf = 6.8; mt = 3.2;
LT = 0.625; Lf = 0.4; Lt = 0.4;
JT = 2.22;  Jf = 1.08;  Jt = 0.93;
lcmT = 0.2; lcmf = 0.163; lcmt = 0.128;
g = 9.81;
linkLengths = [Lt lcmt LT lcmT Lf lcmf].';
linkInertias = [mt Jt mT JT mf Jf g].';
robot_params = [linkLengths; linkInertias];
%ODE Solver Parameters
AT                = 1e-6; % Absolute tolerance for ODE solver
RT                = 1e-3; % Relative tolerance for ODE solver
RF                = 1;    % Refine factor for ODE solver
%% Solve the ODE
load Convergence16.mat
gait_number = 1;
t_temp = []; q_temp = []; dq_temp = []; F_temp = []; u_temp = [];
theta_temp = []; x_stance_temp = [];
t_star = 0; 
x_stance_star = 0;
Xs_plus = Xs_plus + 0.03*rand(10,1); %Add a small offset to the initial conditions

while gait_number < 21
    %Solve ODE
    [t,q,dq,ddq,u,y,dy,F_stance,s_stance,ds_stance,p_swing,dp_swing,~,theta] = solve_stance_ODE(Xs_plus,a_matrix,theta_plus,theta_minus,AT,RT,RF,robot_params);
    %Append final values from ODE
    x_swing    = p_swing(1,:);
    t_temp     = [t_temp; t + t_star]; t_star = t_temp(end); %time increments constantly
    q_temp     = [q_temp q]; %q increments periodically
    dq_temp    = [dq_temp dq];
    theta_temp = [theta_temp; theta];
    F_temp     = [F_temp F_stance];
    u_temp     = [u_temp u];
    
    %increment movement profile of stance leg-end
    x_stance_temp = [x_stance_temp x_stance_star*ones(1,length(t))];
    task_pos = forwardKinematics(Xs_minus(1:5),robot_params);
    x_stance_star = x_stance_star + task_pos(1,6);
    %Record Final values
    Xs_minus = [q(:,end); dq(:,end)];
    %Apply Discrete-time dynamics
    [qs_plus,dqs_plus,F_imp] = discrete_time_dyn(Xs_minus,robot_params);
    Xs_plus = [qs_plus; dqs_plus]; % initial condition at the beginning of the left stance phase
    % Get the initial and final phasing variables
    [theta_plus,~,~]  = phasing_variable(Xs_plus,0);  % initial value for theta
    [theta_minus,~,~] = phasing_variable(Xs_minus,0); % final value for theta
    gait_number = gait_number + 1;
end

% plot the friction cone conditions
mu = 0.9;
Fx = F_temp(1,:);
Fy = F_temp(2,:);
t=t_temp;
plot(t,Fx,t,Fy.*mu)
legend('Fx','Friction Force (Limit)')
grid on
xlabel('Time (s)')
ylabel('Force (N)')

%% Plot joint states, input torque, and GRFs
%Plot the Ground Reaction Force
figure
plot(t_temp,F_temp(1,:),t_temp,F_temp(2,:))
legend('F_x','F_y')
grid on
xlim([0 t_temp(end)])
xlabel('Time (s)')
ylabel('Force (N)')
title('Ground Reaction Force')

%plot input torques
figure
subplot(2,2,1)
plot(t_temp,u_temp(1,:))
title('u_3_1')
grid on
xlim([0 t_temp(end)])
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(2,2,2)
plot(t_temp,u_temp(2,:))
title('u_3_2')
grid on
xlim([0 t_temp(end)])
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(2,2,3)
plot(t_temp,u_temp(3,:))
title('u_4_1')
grid on
xlim([0 t_temp(end)])
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(2,2,4)
plot(t_temp,u_temp(4,:))
title('u_4_2')
grid on
xlim([0 t_temp(end)])
xlabel('Time (s)')
ylabel('Torque (Nm)')

%plot the joint velocities
figure
plot(t_temp,dq_temp(1,:),t_temp,dq_temp(2,:))
legend('dq_3_1','dq_3_2')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
xlim([0 t_temp(end)])
grid on
title('Stance Hip Vel. (dq31) and Swing Hip Vel. (dq32)')
figure
plot(t_temp,dq_temp(3,:),t_temp,dq_temp(4,:))
legend('dq_4_1','dq_4_2')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
xlim([0 t_temp(end)])
grid on
title('Stance Knee Vel. (dq41) and Swing Knee Vel. (dq42)')
figure
plot(t_temp,dq_temp(5,:))
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
xlim([0 t_temp(end)])
grid on
title('Torso Vel. (dq1)')



%plot the joint angles
figure
plot(t_temp,q_temp(1,:),t_temp,q_temp(2,:))
legend('q_3_1','q_3_2')
xlabel('Time (s)')
ylabel('Angle (rad)')
xlim([0 t_temp(end)])
grid on
title('Stance Hip (q31) and Swing Hip (q32) Angles')
figure
plot(t_temp,q_temp(3,:),t_temp,q_temp(4,:))
legend('q_4_1','q_4_2')
xlabel('Time (s)')
ylabel('Angle (rad)')
xlim([0 t_temp(end)])
grid on
title('Stance Knee (q41) and Swing Knee (q42) Angles')
figure
plot(t_temp,q_temp(5,:))
xlabel('Time (s)')
ylabel('Angle (rad)')
xlim([0 t_temp(end)])
grid on
title('Torso Angle (q1)')
%% Plot the Phase Portraits to illustrate stability of the limit cycle
figure
plot(q_temp(1,:),dq_temp(1,:))
%legend('Q_3_1')
grid on
xlabel('q_3_1 (rad)')
ylabel('dq_3_1 (rad/s)')
title('Q31 Phase Portrait')

figure
plot(q_temp(2,:),dq_temp(2,:))
%legend('Q_3_2')
grid on
xlabel('q_3_2 (rad)')
ylabel('dq_3_2 (rad/s)')
title('Q32 Phase Portrait')

figure
plot(q_temp(3,:),dq_temp(3,:))
%legend('Q_4_1')
grid on
xlabel('q_4_1 (rad)')
ylabel('dq_4_1 (rad/s)')
title('Q41 Phase Portrait')

figure
plot(q_temp(4,:),dq_temp(4,:))
%legend('Q_4_2')
grid on
xlabel('q_4_2 (rad)')
ylabel('dq_4_2 (rad/s)')
title('Q42 Phase Portrait')

figure
plot(q_temp(5,:),dq_temp(5,:))
%legend('Q_1')
grid on
xlabel('q_5 (rad)')
ylabel('dq_5 (rad/s)')
title('Q5 Phase Portrait')


%% Create the animation
%Solve the Forward Kinematics
toe1 = []; knee1 = []; hip = []; torsoCM = []; knee2 = []; toe2 = []; torsoTop = [];
p_swing = [x_stance_temp;zeros(1,length(x_stance_temp))];
t = t_temp;
x = q_temp.';
theta = theta_temp;

%process the data to get rid of small time steps
dt = .015;
k=2;
while k < length(t)+1
    if t(k)-t(k-1) < dt
        t(k) = []; x(k,:) = []; theta(k) = []; p_swing(:,k) = [];
        k=k-1;
    end
    k=k+1;
end
%Sove for the robot motion profile in the task space
for k = 1:length(t)
    X = forwardKinematics(x(k,1:5).',robot_params);
    toe1(:,k) = X(:,1)+p_swing(:,k);
    knee1(:,k) = X(:,2)+p_swing(:,k);
    hip(:,k) = X(:,3)+p_swing(:,k);
    torsoCM(:,k) = X(:,4)+p_swing(:,k);
    torsoTop(:,k) = hip(:,k)+[-LT*sin(x(k,5));LT*cos(x(k,5))];
    knee2(:,k) = X(:,5)+p_swing(:,k);
    toe2(:,k) = X(:,6)+p_swing(:,k);
end
% display every dth configuration of x
d=2;
j=1:d:length(t);
% Create the Animation
figure
for i=1:length(j)
    hold off
    plot([toe1(1,j(i)) knee1(1,j(i)) hip(1,j(i)) torsoTop(1,j(i))],[toe1(2,j(i)) knee1(2,j(i)) hip(2,j(i)) torsoTop(2,j(i))],'k',...
        [hip(1,j(i)) knee2(1,j(i)) toe2(1,j(i))],[hip(2,j(i)) knee2(2,j(i)) toe2(2,j(i))],'k',...
        [toe1(1,j(i)) knee1(1,j(i)) hip(1,j(i)) torsoTop(1,j(i))],[toe1(2,j(i)) knee1(2,j(i)) hip(2,j(i)) torsoTop(2,j(i))],'ko',...
        [hip(1,j(i)) knee2(1,j(i)) toe2(1,j(i))],[hip(2,j(i)) knee2(2,j(i)) toe2(2,j(i))],'ko','LineWidth',1.15);
    %scrolling axes
    xlim([hip(1,j(i))-1 hip(1,j(i))+1])
    ylim([-1 2.5])
    H=gca;
    text(H.XLim(1)+.25,-0.5,strcat("Time: ",num2str(t(j(i)),'%4.2f'),'s'))
    text(hip(1,j(i))+1,-0.5,strcat("Theta: ",num2str(rad2deg(theta(j(i))),'%4.2f')))
    axis equal
    xlabel('x (m)')
    ylabel('y (m)')  
    grid on
    hold on
    MM(i)=getframe(gcf);
end
%drawnow limitrate;
text(hip(1,end),2,"END OF SIM",'HorizontalAlignment','center')