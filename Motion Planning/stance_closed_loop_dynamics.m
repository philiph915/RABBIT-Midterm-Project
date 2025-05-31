function [dx] = stance_closed_loop_dynamics(t,x,leg,theta_plus,theta_minus,alpha,robot_params)

% This function calculates the derivative of the state vector for the stance phase.
%
% Inputs:
% t: time
% x: 10-dim state variable during the stance phase
% leg: 0 for the right stance and 1 for the left stance
% theta_plus:  initial value of theta on the orbit
% theta_minus: final value of theta on the orbit
% alpha: coefficient matrix for the bezier polynomial
%
% Outputs:
% dx: 10-dim time derivative of the state vector
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Position and velocity
q  = x(1:5);
dq = x(6:10);

% Dynamic terms
[D,H,B] = stance_phase_dynamic_terms(q,dq,leg,robot_params);

% Controller
[u,h,dh] = normal_controller(x,leg,theta_plus,theta_minus,alpha,robot_params);

% % Constraint matrix for planar motion
% C = zeros(4,9);
% C(1,1) = 1;
% C(2,2) = 1;
% C(3,6) = 1;
% C(4,9) = 1;
% lambda = zeros(4,1);
% 
% if 0 % planar motion
%     lambda = -((((C/D)*C')\C)/D)*(B*u-H);
%     %lambda = -inv(C*inv(D)*C')*C*inv(D)*(B*u-H);
% end

% dx = f(x,u)
%dx = [dq; D\(B*u-H+C'*lambda)];
dx = [dq; -inv(D)*(-B*u+H)];

end

