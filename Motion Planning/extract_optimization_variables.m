function [Xs_plus,Xs_minus,F_imp,a_matrix,theta_plus,theta_minus] = extract_optimization_variables(p,M,robot_params)

% This function extracts the optimization variables from the input vector
% p. Here p is a 2N+2M dimesional vector, where M is the degree of the Bezier 
% polynomial and N is the number of DoF's of the robot.

% Inputs:
%
% p : optimization variables as a 2N+2M dimensional vector
% M : degree of the Bezier polynomial
% 
% Outputs:
% 
% Xs_minus : state variables at the end of the right stance phase
% a_matrix : coefficient matrix for the Bezier polynomial during the right stance phase

%--------------------------------------------------------------------------

% Extract final state vector at the end of the right stance phase
Xs_minus  = p(1:10);
qs_minus  = Xs_minus(1:5);
dqs_minus = Xs_minus(6:10);

% Calculate initial condition at the beginning of the right stance phase
%[qs_plus,dqs_plus,F_imp] = impact_map(qs_minus,dqs_minus,0); % obtain initial condition at the beginning of the left stance
[qs_plus,dqs_plus,F_imp] = discrete_time_dyn(Xs_minus,robot_params);
Xs_plus = [qs_plus; dqs_plus]; % initial condition at the beginning of the left stance phase

% Get the initial and final phasing variables
[theta_plus,~,~]  = phasing_variable(Xs_plus,0);  % initial value for theta 
[theta_minus,~,~] = phasing_variable(Xs_minus,0); % final value for theta 

%disp("Extract_optimization variables; theta_plus; theta_minus;")
%disp(theta_plus); disp(theta_minus);

% Calculate scaled phasing variables and their time derivatives at the beginning
% and end of the right stance phase
%[s_plus,ds_plus,~]   = scaled_phasing_variable(Xs_plus,0,theta_plus,theta_minus);
%[s_minus,ds_minus,~] = scaled_phasing_variable(Xs_minus,0,theta_plus,theta_minus);
[~,ds_plus,~]   = scaled_phasing_variable(Xs_plus,0,theta_plus,theta_minus);
[~,ds_minus,~] = scaled_phasing_variable(Xs_minus,0,theta_plus,theta_minus);

%disp("Extract_optimization variables; s_plus; s_minus;")
%disp(s_plus); disp(s_minus);

% Calculate the first and last two columns of the a matrix
[h0_plus,dh0_plus,~]   = normal_controlled_variables(Xs_plus,0);
[h0_minus,dh0_minus,~] = normal_controlled_variables(Xs_minus,0);

a0   = h0_plus;
aM   = h0_minus;
a1   = a0 + 1/(M * ds_plus)  * dh0_plus;
aM_1 = aM - 1/(M * ds_minus) * dh0_minus;

% Extract middle columns of a and b matrices
a_vec    = p(11:end); % vectorization of middle columns of a
%a_middle = matricization(a_vec,6,M-3);
a_middle = matricization(a_vec,4,2); %4 bezier curves with 2 middle parts each

%Testing:
%disp(a_vec); disp(a_middle); %disp(a0);

a_matrix = [a0 a1 a_middle aM_1 aM];
%disp(a_matrix);
end


