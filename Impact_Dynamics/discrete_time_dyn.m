function [qs_plus,dqs_plus,F_imp] = discrete_time_dyn(x_minus,robot_params)

% Decompose the state variables
qs_minus  = x_minus(1:5);
dqs_minus = x_minus(6:10);

% Floating based coordinates
qe_minus  = [qs_minus;  0; 0];
dqe_minus = [dqs_minus; 0; 0];

% Floating based models
De = extended_inertia_Matrix(qe_minus,robot_params);
Je = extended_swing_Jacobian(qe_minus,robot_params);

% Rigid impact (plastic impact)
A = [De -Je'; Je zeros(2,2)];
B = [De*dqe_minus; zeros(2,1)];
X = inv(A)*B;
%disp("discrete_time_dyn; X");
%disp(X);
dqe_plus = X(1:7); 
dqs_plus  = dqe_plus(1:5); 
F_imp      = X(8:9); 

% Relabeling
%R = [0 1 0; 1 0 0; 0 0 1];
R = [0 1 0 0 0;
     1 0 0 0 0;
     0 0 0 1 0;
     0 0 1 0 0;
     0 0 0 0 1];
qs_plus  = R * qs_minus;
dqs_plus = R * dqs_minus;
x_plus  = [qs_plus; dqs_plus];

end

