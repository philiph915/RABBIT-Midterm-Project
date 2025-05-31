function [t,q,dq,ddq,u,y,dy,F,s,ds,p_swing,dp_swing,p_hip,theta] = solve_stance_ODE(Xs_plus,a_matrix,theta_plus,theta_minus,AT,RT,RF,robot_params)

% This function solves the ODE for the stance phase dynamics.

%--------------------------------------------------------------------------

% SOlve ODE
t_span  = [0 1];%[0 10]; %set maximum time of step
x0      = Xs_plus;
options = odeset('AbsTol',AT,'RelTol',RT,'Refine',RF,'MaxStep',0.01,'events',@stance_phase_events);
warning('off','MATLAB:ode113:IntegrationTolNotMet') %Stop giving me warnings about ODE tolerance not being met
warning('off','MATLAB:ode15s:IntegrationTolNotMet') %Stop giving me warnings about ODE tolerance not being met
t       = [];
x       = [];
%Use ode113 to solve swing phase dynamics
[t,x]   = ode113(@stance_closed_loop_dynamics,t_span,x0,options,0,theta_plus,theta_minus,a_matrix,robot_params);
x_temp  = x;
x       = x_temp';
q       = x(1:5,:);  % Position
dq      = x(6:10,:); % Velocity
%save('ODEresults.mat','x','t');
% Calculate u, y, dy, ddq, GRF, s, ds and p_swing
[~,n]    = size(q);
ddq      = zeros(5,n);
u        = zeros(4,n);
y        = zeros(4,n);
dy       = zeros(4,n);
F        = zeros(2,n);
s        = 0*t;
ds       = 0*t;
theta    = 0*t;
p_swing  = zeros(2,n);
dp_swing = zeros(2,n);
p_hip    = zeros(2,n);
for k=1:n
    [u_temp,y_temp,dy_temp] = normal_controller(x(:,k),0,theta_plus,theta_minus,a_matrix,robot_params);
    u(:,k)   = u_temp;
    y(:,k)   = y_temp;
    dy(:,k)  = dy_temp;
    [ddq_temp,F_temp] = stance_phase_dynamics(q(:,k),dq(:,k),u(:,k),0,robot_params);
    ddq(:,k)  = ddq_temp;
    F(:,k)    = F_temp;
    [s_temp,ds_temp,~,theta_temp] = scaled_phasing_variable(x(:,k),0,theta_plus,theta_minus);
    s(k)      = s_temp;
    ds(k)     = ds_temp;
    theta(k) = theta_temp;
    %Solve for swing leg-end position and velocity
    %Solve forward kinematics to find leg primary points
    task_pos = forwardKinematics(q(:,k),robot_params);
    task_vel = taskVel(q(:,k),dq(:,k),robot_params);
    p_swing(:,k)  = task_pos(:,6);
    dp_swing(:,k) = task_vel(:,6);
    p_hip(:,k) = task_pos(:,3);
    

end % end of loop

end