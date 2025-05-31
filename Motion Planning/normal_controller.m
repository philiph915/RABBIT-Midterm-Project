function [u,h,dh] = normal_controller(x,leg,theta_plus,theta_minus,alpha,robot_params)

% This function calculates the normal controller.
%
% Inputs:
% x: 10-dim state variable during the stance phase
% leg: 0 for the right stance and 1 for the left stance
% theta_plus:  initial value of theta on the orbit
% theta_minus: final value of theta on the orbit
% alpha: coefficient matrix for the bezier polynomial
%
% Outputs:
% u: 4-dim continuous-time controller
% h: 4-dim normal outputs
% dh: 4-dim time derivative of normal outputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Position and velocity
q  = x(1:5);
dq = x(6:10);

% Output function
[h,dh,dh_dq,ddh_ddq] = normal_outputs(x,leg,theta_plus,theta_minus,alpha);

% Normal output function
[~,~,H0] = normal_controlled_variables(x,leg);

% Dynamnic terms
[D,H,B] = stance_phase_dynamic_terms(q,dq,leg,robot_params);

% I/O controller
LgLfh    =  (dh_dq/D)*B;
Lf2h     = -dh_dq*(D\H) + ddh_ddq; 
[KP,KD,epsilon] = Controller_Gains;
u = -LgLfh\Lf2h - LgLfh\(KD/epsilon * dh + KP/epsilon^2 * h);

% % Lie derivatives
% [KP,KD,~] = Controller_Gains;
% Lfh     = dh;
% LgLfh   = dh_dq*inv(D)*B;
% 
% Lf2h    = dh_dq*inv(D)*(-H);
% u       = -(LgLfh)\(Lf2h+KD*Lfh+KP*h);
end

