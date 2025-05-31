function [s,ds,ds_dq,theta] = scaled_phasing_variable(x,leg,theta_plus,theta_minus)
%#codegen
% This function calculates the scaled phasing variable.
%
% Inputs:
% x:   10-dim state variable during the stance phase
% leg: 0 for the right stance and 1 for the left stance
% theta_plus:  initial value of theta on the orbit
% theta_minus: final value of theta on the orbit
%
% Outputs:
% s      phasing variable
% ds:    time derivative of the phasing variable
% ds_dq: 1x5 jacobian matrix 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Phasing variable
[theta,dtheta,dtheta_dq] = phasing_variable(x,leg);

% Scaled phasing variable
s     = (theta - theta_plus)/(theta_minus - theta_plus);
ds    = dtheta/(theta_minus - theta_plus);
ds_dq = dtheta_dq/(theta_minus - theta_plus);

end


