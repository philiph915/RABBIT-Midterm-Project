function [J] = Nonlinear_Cost_MotionPlanning(p,optim_option)

% This function returns the ***cost for the motion planning algorithm.

% Inputs:
%
% p: decision variables of dimension 8*M-2, where M is the degree of the Bezier polynomial
% optim_option: options for the motion planning algorithm
%
% Outputs:
%
% J: value of the cost function

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
robot_params      = optim_option.robot_params;


    %--------------------------------------------------------------------------
    
    % Extract decision variables
    [Xs_plus,Xs_minus,F_imp,a_matrix,theta_plus,theta_minus] = extract_optimization_variables(p,M,robot_params);
    
    % Solve the stance ODE
    [t_stance,q_stance,dq_stance,ddq_stance,u_stance,y_stance,dy_stance,F_stance,s_stance,ds_stance,p_swing,dp_swing,~,~] = solve_stance_ODE(Xs_plus,a_matrix,theta_plus,theta_minus,AT,RT,RF,robot_params);
    %[t_stance,~,~,~,u_stance,~,~,~,~,~,~,~] = solve_stance_ODE(Xs_plus,a_matrix,theta_plus,theta_minus,AT,RT,RF,robot_params);
    %--------------------------------------------------------------------------
    
%     svpath = strcat(pwd,'\ODE Results\');
%     filename = 'ODE_results.mat';
%     save([svpath filename],'q_stance','t_stance');
    
    % Step length
    %step_length = p_swing(1,end); %changed to reflect x-coordinate of swing leg
    step_time = t_stance(end);
    sum_stance = 0;
    %Calculate the cost by integrating the commanded torque over one step
    for k = 1:length(t_stance)-1 %skip the last point because that is where impact occurs?
        sum_stance = sum_stance + norm(u_stance(:,k),2)^2 * (t_stance(k+1)-t_stance(k));
    end
    J = sum_stance/step_time;
    
    %J = 1;
    
    %J = F_min_v + max(-F_stance(2,:)); %try using GRF as cost function
    %J = 300 + max(-F_stance(2,:)); %try using GRF as cost function
    %J = F_min_v -F_stance(2,round(length(F_stance)*0.5));
end  
    %Calculate cost
    
%     if 0%0
%         % Calculate cost
%         sum_stance = 0;
%         for k=1:length(t_stance)-1
%             sum_stance = sum_stance + norm(u_stance(:,k),2)^2 * (t_stance(k+1) - t_stance(k));
%         end % end of for
%         J = sum_stance; %/step_length;
%     else % CMT
%         % Calculate cost
%         %B   = Cfcn_Robot_Input_Matrix_Right_Parametric(zeros(9,1),zeros(9,1));
%         %B = input_Matrix(Xs_plus(1:5),robot_params); %don't know if I should use Xs_plus or Xs_minus
%         
%         B = [eye(4);zeros(1,4)]; %B matrix is always the same
%         dqa = B' * dq_stance;
%         [m,~] = size(dqa);
%         sum_stance = 0;
%         for k=1:length(t_stance)-1
%             for j=1:m
%                 sum_stance = sum_stance + max(u_stance(j,k)*dqa(j,k),0) * (t_stance(k+1) - t_stance(k));
%             end % end of for
%         end % end of for
%         %[~,~,~,~,~,m1,m2,m3,mT,~,~,~,~,~,~,~,~,~,~,~,~] = Human_Body_Parameters;
%         %m_tot = 2*(m1 + m2 + m3) + mT;
%         m_tot = 2*(robot_params(7)+robot_params(11))+robot_params(9);
%         g = 9.81;
%         J = sum_stance/(m_tot*g*step_length);
%         
%         %step_length   = p_swing(2,end);
%         %ave_velocity  = step_length/t_stance(end);
%         %J = -ave_velocity;
%     end
% else
%     J = 1;
% end

%end


