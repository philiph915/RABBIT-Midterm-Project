function [theta,dtheta,dtheta_dq] = phasing_variable(x,leg)
%#codegen
% This function calculates the phasing variable.
%
% Inputs:
% x:   10-dim state variable during the stance phase
% leg: 0 for the right stance and 1 for the left stance
%
% Outputs:
% theta:     phasing variable
% dtheta:    time derivative of the phasing variable
% dtheta_dq: 1x5 jacobian matrix 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Position and velocity
q  = x(1:5);
dq = x(6:10);

% Matrices
% q = [q31  q32  q41  q42  q1].'; 
% theta = 3*pi/2 + q41/2 - q1 - q31;
c1 = [-1 0 1/2 0 -1];
c0 = 3*pi/2;

theta     = c1*q+c0;
dtheta    = c1*dq;
dtheta_dq = c1;

end

