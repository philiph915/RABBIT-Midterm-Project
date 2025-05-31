function [D,H,B] = flight_phase_dynamic_terms(q,dq)
%#codegen
% This function generates the dyanmic terms during the flight phase.

% Inputs:
%
% q:  12-dim position
% dq: 12-dim velocity

% Outputs
%
% D: 12x12 mass-inertia matrix
% H: 12-dim Coriolis and gravity terms vector
% B: 12x6 input matrix

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Mass inertia
D    = Cfcn_Robot_Mass_Inertia_Hip_Parametric(q,dq);

% Coriolis
CdqT = Cfcn_RobotT_Coriolis_Centrifugal_Terms_Hip_Parametric(q,dq);
CdqR = Cfcn_RobotR_Coriolis_Centrifugal_Terms_Hip_Parametric(q,dq);
CdqL = Cfcn_RobotL_Coriolis_Centrifugal_Terms_Hip_Parametric(q,dq);
Cdq  = CdqT + CdqR + CdqL;

% Gravity
G    = Cfcn_Robot_Gravity_Terms_Hip_Parametric(q,dq);

% Coriolis and gravity
H    = Cdq + G;

% Input matrix
B    = Cfcn_Robot_Input_Matrix_Hip_Parametric(q,dq);

end

