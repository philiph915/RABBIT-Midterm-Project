%% ME6984 Feedback Control of Dynamic Legged Locomotion 
% Midterm Project Part 1
% By Philip Hancock
% 4/2/2019
%
% Utilize HZD-based trajectory optimization techniques to design a 
% walking gait for the 5-link biped robot at a speed of 0.8 m/s. 
%% 
%clc
%clear all

% Optimization parameters
optim_option = [];

% Specify Bezier polynmoial order
M = 5;
optim_option.M = M;

% Specifty robot parameters

% Values taken from journal paper
mT = 20; mf = 6.8; mt = 3.2;
LT = 0.625; Lf = 0.4; Lt = 0.4;
JT = 2.22;  Jf = 1.08;  Jt = 0.93;
lcmT = 0.2; lcmf = 0.163; lcmt = 0.128;
g = 9.81;

linkLengths = [Lt lcmt LT lcmT Lf lcmf].';
linkInertias = [mt Jt mT JT mf Jf g].';
robot_params = [linkLengths; linkInertias];


% Specify minimum and maximum of joint angles in (deg)
%Generalized Coordinates (q31, q32, q1 CCW positive; q41, q42 CW positive)
%q31 = stance leg thigh wrt torso
%q41 = stance leg shin wrt stance leg thigh
%q32 = swing leg thigh wrt torso
%q42 = swing leg shin wrt swing leg thigh
%q1 = absolute angle of torso wrt vertical
%q = [q31  q32  q41  q42  q1].'; %definition of joint position vector

%Femur joint limits
qf_min = 160;%90;
qf_max = 260;%270;
%Tibya joint limits
qt_min = 0;
qt_max = 45;
%Torso Joint Limits
qT_min = -30;%-80;
qT_max = 0;%0;

% Calculate the position bounds
q_min_stance = pi/180 * [qf_min; qf_min; qt_min; qt_min; qT_min];
q_max_stance = pi/180 * [qf_max; qf_max; qt_max; qt_max; qT_max];
optim_option.q_min_stance = q_min_stance;
optim_option.q_max_stance = q_max_stance;

% Specify maximum angular velocity in (rad/s)
dq_max = deg2rad(120);

% Calculate the velocity bounds
dq_min_stance = -dq_max * ones(length(q_min_stance),1);
dq_max_stance =  dq_max * ones(length(q_min_stance),1);
optim_option.dq_min_stance = dq_min_stance;
optim_option.dq_max_stance = dq_max_stance;

% Specify other parameters
ds_min            = 0.1;  % The minimum value for the time deriavtive of the sacled phasing varibale
u_max             = 2000;%70;%60;%2000;  % maximum torque values for DC motors in (Nm)

F_min_v           = 10;%10;   % mimunum value for the vertical component of the GRF during stance phase in (N)

F_min_v_impulsive = 0.1;  % minumum value for the vertical component of the impulsive GRF during flight to stance impact in (N)
mu_s              = 0.9;  % friction coefficient between the leg end and ground
step_length_min   = 0.1;  % minimum step length in (m)
ave_velocity_min  = 0.8;  % minimum average velocity of the robot in (m/s)
AT                = 1e-6; % Absolute tolerance for ODE solver
RT                = 1e-3; % Relative tolerance for ODE solver
RF                = 1;    % Refine factor for ODE solver
p_swing_h_min     = 0.1;
t_s_min           = 0.2;
vcm_min           = 0.5;

% Calculate other parameters
optim_option.ds_min            = ds_min;
optim_option.u_max             = u_max;
optim_option.F_min_v           = F_min_v;
optim_option.F_min_v_impuslive = F_min_v_impulsive;
optim_option.mu_s              = mu_s;
optim_option.AT                = AT;
optim_option.RT                = RT;
optim_option.RF                = RF;
optim_option.step_length_min   = step_length_min;
optim_option.ave_velocity_min  = ave_velocity_min;
optim_option.p_swing_h_min     = p_swing_h_min;
optim_option.t_s_min           = t_s_min;
optim_option.vcm_min           = vcm_min;
optim_option.robot_params      = robot_params;

% Parallel computation enabled
%matlabpool(24);

%--------------------------------------------------------------------------
%%
% Optimization

%SW = warning('off', 'MATLAB:ode45:IntegrationTolNotMet');
%SW = warning('off', 'MATLAB:ode113:IntegrationTolNotMet');
warning('off','MATLAB:ode113:IntegrationTolNotMet') %Stop giving me warnings about ODE tolerance not being met


% fmincon options
MinimizationOptions = optimset('OutputFcn',@outfun,...
    'MaxFunEvals',1e6,...
    'TolCon',1e-3,...
    'TolFun',1e-4,...
    'TolX',1e-15,...%1e-15;
    'Algorithm','active-set',...
    'AlwaysHonorConstraints','bounds',...
    'Display','iter-detailed',...
    'FunValCheck','on',...
    'DiffMaxChange', 0.1,...%0.01
    'DiffMinChange', 1e-2,...%1e-4
    'MaxIter', 1000,...
    'UseParallel','always');

% Initial condition for the optimizer
which_initial_condition = 'random';
EXITFLAG = -2;

% Lower and upper bounds
a_min  = pi/180 * [qf_min; qf_min; qt_min; qt_min];
a_max  = pi/180 * [qf_max; qf_max; qt_max; qt_max];
a_lb   = repeat_vector(a_min,2); %appends a copy of the vector to itself
a_up   = repeat_vector(a_max,2);
lb     = [q_min_stance; dq_min_stance; a_lb];
ub     = [q_max_stance; dq_max_stance; a_up];

%%
while EXITFLAG==-2
    
    switch which_initial_condition
        case 'random'
            nX = 10 + 8;
            X0 = zeros(nX,1);
            
            %From Textbook pg. 183
            X0(1:5)  = [2.9; 3.55; 0.25; 0.42; -0.055];
            X0(6:10) = [-0.5; 0.1; 0.3; 1.3; -0.5];
            
            %X0(1:5)  = X0(1:5) + 0.1*randn(5,1);
            
            %Relabeled legs
            %X0(1:5)  = [2.91; 3.55; 0.26; 0.42; .2];
            %X0(1:5)  = X0(1:5) + 0.01*randn(5,1);
            %X0(6:10) = [-0.3; 0.4; 0.3; 2.1; -0.5];
            %X0(6:10)  = X0(6:10) + 0.01*randn(5,1);
            X0(11:end) = randn(8,1); %random values for bezier coefficients
            
            %Guess Initial Posittion
            %Progressively Straighter Legs
            %X0(1:5) = [3.141592653589793   3.608241335695517   0.087266462599716   0.174532925199433  -0.174532925199433].';
            %X0(1:5) = [3.054326190990077   3.490658503988659   0.087266462599716   0.087266462599716  -0.087266462599716].';
            %X0(1:5) = [3.141592653589793   3.403392041388943   0.087266462599716   0.087266462599716  -0.087266462599716].';                      
            
            %Bent Stance Knee
            %X0(1:5) = [3.4907 4.132714346716140 0.2618 0.5236 -0.5236].'; %a configuration that starts with swing leg end approxiamtely at y=0 (up to 10^-8)
            
            %Guess Initial Velocity 
            %X0(6:10) = [-0.1690 -0.2408 -1.2115 -0.9790 -1.3194].'; + 0.03*randn(5,1);
            
            %Load a good starting point, add some variations
            load Convergence16.mat; X0 = [Xs_minus; a_matrix(:,3); a_matrix(:,4)];
            X0(8) = -X0(8)-0.1;
            X0(3) = X0(3)/10;
            %X0(6:10) = [-0.9690 0.2408 0.2115 0.5790 -1.0194].'; %Velocity Guess
            
            %X0(6:10) = X0(6:10) + 0.1*randn(5,1); %add variation to velocity IC's only
            %X0([3,7]) = randn(2,1); %random values for stance knee bezier coefficients
            %X0(11:end) = randn(8,1); %random values for all bezier coefficients
            X0 = X0 + .03*randn(18,1); %add randomness to all IC's
            
            
            %             X0 = [3.2764; 3.6725; 0.6448; 0.4014; -0.0098;
            %                 -0.0087; -0.2224; 1.4567; 1.1370; -0.3602;
            %                 3.5209; 3.6339; 0.1564; 0.3880;
            %                 3.3213; 3.8068; 0.1622; 0.1708];
    end % end of witch
    
    % fmincon
    [X,FVAL,EXITFLAG,OUTPUT,LAMBDA,GRAD,HESSIAN] = fmincon(@Nonlinear_Cost_MotionPlanning,X0,[],[],[],[],lb,ub,@Nonlinear_Constraints_MotionPlanning,MinimizationOptions,optim_option);
    [Xs_plus,Xs_minus,F_imp,a_matrix,theta_plus,theta_minus] = extract_optimization_variables(X,M,robot_params);
    
    %EXITFLAG=0;
    
end % end of while

% Close matlabpool
%matlabpool close
save('ConvergenceX.mat','a_matrix','Xs_minus','Xs_plus','theta_plus','theta_minus','F_imp')
%% Animate the optimized gait
%Solve the Forward Kinematics
toe1 = []; knee1 = []; hip = []; torsoCM = []; knee2 = []; toe2 = [];
load ODE_results.mat
t1 = t_stance1;
x1 = q_stance1.';
for k1 = 1:length(t1)
    X1 = forwardKinematics(x1(k1,1:5).',robot_params);
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
j=1:d:length(t1);
% Create the Animation
figure(1)
%tic; 
for i=1:length(j)
hold off
%plot(xd(1),xd(2),'kx',[x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'ko',0,0,'k^', ...
%    [0 x1(j(i))],[0 y1(j(i))],'k',[x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'k','LineWidth',1.15)
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
MM(i)=getframe(gcf);
end
drawnow;
text(0,1.5,"END OF SIM",'HorizontalAlignment','center')
%toc %Display total time taken to run animation (Should be ~=T_f
figure (2)
subplot(2,1,1)
plot(t1,u_stance1,'LineWidth',1.4)
legend('stance hip','swing hip','stance knee','swing knee')
grid on
xlim([0 t1(end)])
xlabel('Time (s)')
ylabel('Torque (Nm)')
subplot(2,1,2)
plot(t1,F_stance1(1,:),t1,F_stance1(2,:),'LineWidth',1.4)
legend('Fx','Fy')
grid on
xlim([0 t1(end)])
xlabel('Time (s)')
ylabel('Force (N)')
disp(Xs_minus)
%% end of code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
