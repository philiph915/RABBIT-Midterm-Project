function [ddqs,F] = stance_phase_dynamics(qs,dqs,u,leg,robot_params)

% This function generates the acceleration and GRF during the stance
% phases.

% Inputs:
%
% qs:  5-dim position
% dqs: 5-dim velocity
% u:   4-dim control inputs
% leg: scalar quantity (0 for the right stance phase and 1 for the left stance phase)

% Outputs
%
% ddqs: 5-dim acceleration
% F:    2-dim GRF

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Lift map
%task_pos = forwardKinematics(qs,robot_params);
%p0 = task_pos(:,1);

%Extended Coordinates
%assume the stance leg is always at the origin!
qe = [qs;0;0];
dqe = [dqs;0;0];

De = extended_inertia_Matrix(qe,robot_params);
Cedq = extended_Coriolis_Centrifugal(qe,dqe,robot_params);
Ge = extended_gravity_terms(qe,robot_params);
He = Cedq + Ge;
Be = extended_input_Matrix(qe,robot_params);
Je = [zeros(2,5) eye(2)]; %stance leg-end Jacobian 

%A = [De zeros(7,2); zeros(7,2) -Je.'];
%RHS = [Be*u - He;zeros(7,1)];
A = [De -Je.';Je zeros(2,2)];
%disp(A);
RHS = [Be*u - He;zeros(2,1)];
%disp(RHS);
X = A\RHS;
ddqs = X(1:5); %only care about the joint accelerations
F = X(8:9); %2D ground reaction force [Fx;Fy]
%disp(X);
end
    


% if leg==0 % right stance
%     [pcm p0 p0T pHR p1R p2R p3R pHL p1L p2L p3L] = Cfcn_Robot_Primary_Points_Right(qs);
%     [vcm J_cm dJ_cm v0 J0 dJ0 v0T vHL vHR] = Cfcn_Robot_VelAccel_Right(qs,dqs);
% else % left stance
%     [pcm p0 p0T pHR p1R p2R p3R pHL p1L p2L p3L] = Cfcn_Robot_Primary_Points_Left(qs);
%     [vcm J_cm dJ_cm v0 J0 dJ0 v0T vHL vHR] = Cfcn_Robot_VelAccel_Left(qs,dqs);
% end % end of if

% % Extended coordinates
% q  = [p0; qs];
% dq = [J0; eye(5,5)]*dqs;
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
% if 1
%     % Constrained dyanmics
%     A   = [D -J'; J zeros(3,3)];
%     RHS = [B*u-H; -dJ*dq];
%     X    = A\RHS;
% else % planar model
%     C = zeros(5,12);
%     C(1,1)  = 1;
%     C(2,4)  = 1;
%     C(3,5)  = 1;
%     C(4,9)  = 1;
%     C(5,12) = 1;
%     J_temp  = [J; C];
%     J       = J_temp;
%     dJ_temp = [dJ; zeros(5,12)];
%     dJ      = dJ_temp;
%     A   = [D -J'; J zeros(8,8)];
%     RHS = [B*u-H; -dJ*dq];
%     X    = A\RHS;
% end
% 
% % Projection to find acceleration
% ddqs = X(4:12);
% 
% % GRF
% F    = X(13:15);
% 
% end

