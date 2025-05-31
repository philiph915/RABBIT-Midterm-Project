function [h0,dh0,dh0_dq] = normal_controlled_variables(x,leg)

% This function calculates the controlled variable.
%
% Inputs:
% x:   10-dim state variable during the stance phase
% leg: 0 for the right stance and 1 for the left stance
%
% Outputs:
% h0:     4-dim controlled variables
% dh0:    4-dim time derivative of controlled varaibles
% dh0_dq: 4x5 Jacobian matrix
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Position and velocity
q  = x(1:5);
dq = x(6:10);

% Controlled varaibles
%I  = eye(5,5);
%H0 = I(2:end,:);
H0 = [eye(4) zeros(4,1)];

% q  =     [q31 q32 q41 q42 q1]; therefore:
%
% H0 =     1     0     0     0     0
%          0     1     0     0     0
%          0     0     1     0     0
%          0     0     0     1     0

%I don't think I need this part
% if leg==0 % right stance
%     % do nothing ...
% else % left stance
%     [Sqs,Sqf,Sxs,Sxf] = state_symmetry_matrices;
%     H0 = H0 * Sqs;
% end % end of if

h0     = H0 * q;
dh0    = H0 * dq;
dh0_dq = H0;

end

