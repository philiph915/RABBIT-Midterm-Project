function [Cineq,Ceq] = Nonlinear_Constraints_MotionPlanning(p,optim_option)

% This function returns the inequality and equality constraints for the
% motion planning algorithm.

% Inputs:
%
% p: decision variables of dimension 8*M-2, where M is the degree of the Bezier polynomial
% optim_option: options for the motion planning algorithm
%
% Outputs:
%
% Cineq: inequality constraints
% Ceq:   equality constraints

%--------------------------------------------------------------------------

% Extract optimization parameters
M                 = optim_option.M;
AT                = optim_option.AT;
RT                = optim_option.RT;
RF                = optim_option.RF;
ds_min            = optim_option.ds_min;
q_min_stance      = optim_option.q_min_stance;
q_max_stance      = optim_option.q_max_stance;
dq_min_stance     = optim_option.dq_min_stance;
dq_max_stance     = optim_option.dq_max_stance;
u_max             = optim_option.u_max;
F_min_v           = optim_option.F_min_v;
F_min_v_impuslive = optim_option.F_min_v_impuslive;
mu_s              = optim_option.mu_s;
step_length_min   = optim_option.step_length_min;
ave_velocity_min  = optim_option.ave_velocity_min;
p_swing_h_min     = optim_option.p_swing_h_min;
t_s_min           = optim_option.t_s_min;
vcm_min           = optim_option.vcm_min;
robot_params      = optim_option.robot_params;

%--------------------------------------------------------------------------

% Extract decision variables
[Xs_plus,Xs_minus,F_imp,a_matrix,theta_plus,theta_minus] = extract_optimization_variables(p,M,robot_params);

% Solve the stance ODE
[t_stance,q_stance,dq_stance,ddq_stance,u_stance,y_stance,dy_stance,F_stance,s_stance,ds_stance,p_swing,dp_swing,p_hip,theta] = solve_stance_ODE(Xs_plus,a_matrix,theta_plus,theta_minus,AT,RT,RF,robot_params);

svpath = strcat(pwd,'\ODE Results\');
filename = 'ODE_results.mat';
q_stance1 = q_stance; dq_stance1 = dq_stance; t_stance1 = t_stance; u_stance1 = u_stance;
p_swing1 = p_swing; theta_plus1 = theta_plus; theta_minus1 = theta_minus;
s_stance1 = s_stance; ds_stance1 = ds_stance; theta1 = theta;
F_stance1 = F_stance;
save([svpath filename],'q_stance1','dq_stance1','t_stance1','u_stance1','p_swing1','theta_plus1','theta_minus1','s_stance1','ds_stance1','theta1','F_stance1','a_matrix');

%--------------------------------------------------------------------------
% Inequality constraints
Cineq = [];

% Calculations for Foot clearance
%Check the position of the swing leg-end at the start/end of the gait
task_pos = forwardKinematics(Xs_plus(1:5),robot_params);
p_swing_initial = task_pos(:,6);
task_pos2 = forwardKinematics(Xs_minus(1:5),robot_params);
p_swing_final = task_pos2(:,6);

%**************************************************************************
% Feasibilty of GRF (friction cone conditions)

% % %Check that GRF is always upwards
 Cineq = [Cineq; F_min_v + max(-F_stance(2,:))];
% % %Friction Cone Condition
 Cineq = [Cineq; max(abs(F_stance(1,:)) - mu_s * abs(F_stance(2,:)))];
% 
% Cineq = [Cineq; F_min_v - F_stance(2,end)]; %positive GRF at end
% Cineq = [Cineq; F_min_v - F_stance(2,1)]; %positive GRF at start

%**************************************************************************

% Feasibility of theta limits
Cineq = [Cineq; theta_plus - theta_minus];

% Feasibility of impact force (friction cone conditions)
F_impulsive_v = F_imp(2);
F_impulsive_h = F_imp(1);
Cineq = [Cineq;  F_min_v_impuslive - F_impulsive_v];
Cineq = [Cineq;  F_impulsive_h - mu_s * F_impulsive_v];
Cineq = [Cineq; -F_impulsive_h - mu_s * F_impulsive_v];

% Feasibility of Bezier matrix
Cineq = [Cineq; max(a_matrix')'     - q_max_stance(1:4)];
Cineq = [Cineq; q_min_stance(1:4) - min(a_matrix')'];

% Strictly increasing pahsing variable
Cineq = [Cineq; ds_min - min(ds_stance)];

% Feasibilty of positions
Cineq = [Cineq; q_min_stance - min(q_stance')'];
Cineq = [Cineq; max(q_stance')' - q_max_stance];
%******************************************************************
% Feasibilty of velocity
Cineq = [Cineq; dq_min_stance - min(dq_stance')'];
Cineq = [Cineq; max(dq_stance')' - dq_max_stance];
%******************************************************************
% Feasibilty of torque
%changed to match dimension of m
Cineq = [Cineq; -u_max*ones(4,1) - min(u_stance')'];
Cineq = [Cineq;  max(u_stance')' - u_max*ones(4,1)];

% Step length and velocity requiremnets
step_length   = p_swing(1,end);
ave_velocity  = step_length/t_stance(end);
Cineq = [Cineq; step_length_min - step_length];
Cineq = [Cineq; ave_velocity_min - ave_velocity];

Cineq = [Cineq; p_swing_initial(1) + p_swing_h_min]; %swing leg-end must start at or behind x=-0.1m
Cineq = [Cineq; p_swing_h_min      - p_swing_final(1)]; %swing leg-end must finish past x=0.1m
Cineq = [Cineq; min(p_swing(2,:))]; %y-coordinate of swing leg-end must reach 0
Cineq = [Cineq; -min(p_swing(2,:))]; %swing leg cannot go below zero during gait
Cineq = [Cineq; dp_swing(2,end)]; %swing leg-end must be moving down at the end of the gait
Cineq = [Cineq; 0.02-max(p_swing(2,:))]; %swing leg-end must come up to at least 0.02m above ground

% Time constraint
Cineq = [Cineq; t_s_min - t_stance(end)]; %step time must be at least 0.2s

%--------------------------------------------------------------------------

% Equality constarints
Ceq = [];

% Periodicity condition
EQ_Periodicity = [q_stance(:,end); dq_stance(:,end)] - Xs_minus;
Ceq = [Ceq; EQ_Periodicity(1:5)];
Ceq = [Ceq; EQ_Periodicity(6:10)];

% Impact condition
Ceq = [Ceq; p_swing_final(2)];  %Changed to reflect y-coordinate

end


