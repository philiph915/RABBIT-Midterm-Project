function [h,dh,dh_dq,ddh_ddq] = normal_outputs(x,leg,theta_plus,theta_minus,alpha)

% This function calculates the controlled variable.
%
% Inputs:
% x: 10-dim state variable during the stance phase
% leg: 0 for the right stance and 1 for the left stance
% theta_plus:  initial value of theta on the orbit
% theta_minus: final value of theta on the orbit
% alpha: coefficient matrix for the bezier polynomial, 6x(M+1)-dim matrix
%
% Outputs:
% h: 4-dim normal outputs
% dh: 4-dim time derivative of normal outputs
% dh_dq: 4x5 Jacobian matrix
% ddh_ddq: 4x1 second-order Jacobian
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Normal controlled variables
[h0,dh0,dh0_dq] = normal_controlled_variables(x,leg);
H0 = dh0_dq;

% Scaled phasing variabls
[s,ds,ds_dq] = scaled_phasing_variable(x,leg,theta_plus,theta_minus);

% Desired evolution
hd      = bezier(alpha,s);
dhd     = bezierd(alpha,s) * ds;
dhd_dq  = bezierd(alpha,s) * ds_dq;

% Normal outputs
h       = h0 - hd;
dh      = dh0 - dhd;
dh_dq   = H0 - dhd_dq; 
ddh_ddq = -beziera(alpha,s) * (ds)^2;

end


