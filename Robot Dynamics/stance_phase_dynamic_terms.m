function [Ds,Hs,Bs] = stance_phase_dynamic_terms(qs,dqs,~,robot_params)
% ***replaced leg with ~
% This function generates the dyanmic terms during the stance
% phases.

% Inputs:
%
% qs:   5-dim position
% dqs:  5-dim velocity
% leg: scalar quantity (0 for the right stance phase and 1 for the left stance phase)

% Outputs
%
% Ds: 5x5 mass-inertia matrix
% Hs: 5-dim Coriolis and gravity terms vector
% Bs: 5x4 input matrix

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ds = inertia_Matrix(qs,robot_params);

Cdq = Coriolis_Centrifugal(qs,dqs,robot_params);
G = gravity_terms(qs,robot_params);
Hs = Cdq + G;

Bs = input_Matrix(qs,robot_params);


% % Lift map
% if leg==0 % right stance
%     [pcm p0 p0T pHR p1R p2R p3R pHL p1L p2L p3L] = Cfcn_Robot_Primary_Points_Right(qs);
%     [vcm J_cm dJ_cm v0 J0 dJ0 v0T vHL vHR] = Cfcn_Robot_VelAccel_Right(qs,dqs);
% else % left stance
%     [pcm p0 p0T pHR p1R p2R p3R pHL p1L p2L p3L] = Cfcn_Robot_Primary_Points_Left(qs);
%     [vcm J_cm dJ_cm v0 J0 dJ0 v0T vHL vHR] = Cfcn_Robot_VelAccel_Left(qs,dqs);
% end % end of if
% 
% % Extended coordinates
% q  = [p0; qs];
% dq = [J0; eye(9,9)]*dqs;
% 
% % Flight dynamics
% [D,H,B] = flight_phase_dynamic_terms(q,dq);
% [v1R v1L J1R J1L dJ1R dJ1L] =  Cfcn_Robot_VelAccel_LegEnds_Hip(q,dq);
% 
% % Leg Jacoian matrices
% if leg==0 % right stance
%     J  = J1R;
%     dJ = dJ1R;
% else % left stance
%     J  = J1L;
%     dJ = dJ1L;
% end % end of if
% 
% % Projection matrix
% proj = (eye(12,12) - J'*(((J/D)*J')\J)/D);
% 
% % Consterained H and B
% H_const = proj * H + J'*(((J/D)*J')\dJ)*dq;
% 
% % Constrained B
% B_const = proj * B;
% 
% % Projection to reduced coordinates
% Ds = [J0' eye(9,9)] * D * [J0; eye(9,9)];
% Hs = [J0' eye(9,9)] * (D * [dJ0*dqs; zeros(9,1)] + H_const);
% %Bs = [J0' eye(11,11)] * B_const;
% Bs =  B(4:12,:); % For numerical purposes
% 
% end

