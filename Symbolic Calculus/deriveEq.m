%% Derive Equations of Motion using Symbolic Calculus
%ME6984 HW1
%By Philip Hancock
clear; clc; close all;
%Specify joint variables
%1 = stance leg; 2 = swing leg
%3 = thigh (femur); 4 = shin  (tibya)
syms q31  q32  q41  q42  q1  real;
syms dq31 dq32 dq41 dq42 dq1 real;

%Generalized Coordinates (q31, q32, q1 CCW positive; q41, q42 CW positive)
%q31 = stance leg thigh wrt torso
%q41 = stance leg shin wrt stance leg thigh
%q32 = swing leg thigh wrt torso
%q42 = swing leg shin wrt swing leg thigh
%q1 = absolute angle of torso wrt vertical
q = [q31  q32  q41  q42  q1].'; 
dq = [dq31 dq32 dq41 dq42 dq1].';

%Absolute Coordinates (positive CCW, measured WRT vertical)
syms th_31 th_32 th_41 th_42 th_1
th_31 = q1+q31; th_32=q1+q32; th_41=q1+q31-q41; th_42=q1+q32-q42; th_1=q1;
dth_31 = dq1+dq31; dth_32=dq1+dq32; dth_41=dq1+dq31-dq41; dth_42=dq1+dq32-dq42; dth_1=dq1;

%Absolute angles
th = [th_31; th_32; th_41; th_42; th_1];
%Absolute angular velocities
dth = [dth_31; dth_32; dth_41; dth_42; dth_1];

%Define Robot Parameters
syms Lt lcmt LT lcmT Lf lcmf;
linkLengths = [Lt lcmt LT lcmT Lf lcmf].';
syms mt Jt mT JT mf Jf g;
linkInertias = [mt Jt mT JT mf Jf g].';
robot_params = [linkLengths; linkInertias];

%Define CoM Positions [x;y]
p_toe1 = [0;0]; %stance leg-end
pcmt1 = [(Lt - lcmt)*sin(th_41); -(Lt - lcmt)*cos(th_41)]; %stance shin CoM 
p_knee1 = [Lt*sin(th_41); -Lt*cos(th_41)]; %stance knee
pcmf1 = p_knee1 + [(Lf-lcmf)*sin(th_31);-(Lf-lcmf)*cos(th_31)]; %stance thigh CoM
p_hip = p_knee1 + [(Lf)*sin(th_31);-(Lf)*cos(th_31)]; %hip
pcmT = p_hip +[-lcmT*sin(th_1);lcmT*cos(th_1)]; %torso CoM
pcmf2 = p_hip + [-lcmf*sin(th_32);lcmf*cos(th_32)]; %swing thigh CoM
p_knee2 = p_hip + [-Lf*sin(th_32);Lf*cos(th_32)]; %swing knee
pcmt2 = p_knee2 + [-lcmt*sin(th_42);lcmt*cos(th_42)]; %swing shin CoM
p_toe2 = p_knee2 + [-Lt*sin(th_42);Lt*cos(th_42)]; %swing leg-end
task_pos = [p_toe1 p_knee1 p_hip pcmT p_knee2 p_toe2]; %task space configuration coordinates

%Calculate CoM Velocities
Jcmt1 = jacobian(pcmt1,q); Jcmt1 = simplify(Jcmt1); vcmt1 = Jcmt1*dq;   vcmt1 = simplify(vcmt1);
Jcmf1 = jacobian(pcmf1,q); Jcmf1 = simplify(Jcmf1); vcmf1 = Jcmf1*dq;   vcmf1 = simplify(vcmf1);
JcmT = jacobian(pcmT,q); JcmT = simplify(JcmT); vcmT = JcmT*dq;          vcmT = simplify(vcmT);
Jcmf2 = jacobian(pcmf2,q); Jcmf2 = simplify(Jcmf2); vcmf2 = Jcmf2*dq;   vcmf2 = simplify(vcmf2);
Jcmt2 = jacobian(pcmt2,q); Jcmt2 = simplify(Jcmt2); vcmt2 = Jcmt2*dq;   vcmt2 = simplify(vcmt2);
Jtoe2 = jacobian(p_toe2,q); Jtoe2 = simplify(Jtoe2); vtoe2 = Jtoe2*dq;  vtoe2 = simplify(vtoe2);

task_vel = [vcmt1 vcmf1 vcmT vcmf2 vcmt2 vtoe2]; %task space velocities


%Calculate Kinetic Energy
K = 1/2*mt*transpose(vcmt1)*vcmt1+1/2*Jt*dth_41^2 + ... 
    1/2*mf*transpose(vcmf1)*vcmf1+1/2*Jf*dth_31^2 + ...
    1/2*mT*transpose(vcmT)*vcmT+1/2*JT*dth_1^2 + ...
    1/2*mf*transpose(vcmf2)*vcmf2+1/2*Jf*dth_32^2 + ... 
    1/2*mt*transpose(vcmt2)*vcmt2+1/2*Jt*dth_42^2;
K = simplify(K);

%Calculate Potential Energy
V = g*(mt*pcmt1(2)+mf*pcmf1(2)+mT*pcmT(2)+mt*pcmt2(2)+mf*pcmf2(2));
V = simplify(V);

%Find Mass-Inertia Matrix
dK_dqdot = jacobian(K,dq);
D        = jacobian(dK_dqdot,dq);
D        = simplify(D);
%Find Coriolis and Centrifugal terms
C = 0*D;
N = length(q);
for k=1:N
    for j=1:N
        sum = 0*g;
        for i=1:N
            sum = sum + 1/2*(diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)))*dq(i);
        end
        C(k,j) = sum;
    end
end
Cdq = C*dq;
Cdq = simplify(Cdq);
%Find Gravitational Terms
G = jacobian(V,q).';
%Input Matrix
B = jacobian(q([1:4]),q).'; 
%% Impact Dynamics
syms x y real;
syms dx dy real;
qe = [q;x;y];
dqe = [dq;dx;dy];

%Define positions in terms of extended coordinates
p_toe1e = [x;y]; %stance leg-end
pcmt1e = [x;y] + [(Lt - lcmt)*sin(th_41); -(Lt - lcmt)*cos(th_41)]; %stance shin CoM 
p_knee1e = [x;y] + [Lt*sin(th_41); -Lt*cos(th_41)]; %stance knee
pcmf1e = p_knee1e + [(Lf-lcmf)*sin(th_31);-(Lf-lcmf)*cos(th_31)]; %stance thigh CoM
p_hipe = p_knee1e + [(Lf)*sin(th_31);-(Lf)*cos(th_31)]; %hip
pcmTe = p_hipe +[-lcmT*sin(th_1);lcmT*cos(th_1)]; %torso CoM
pcmf2e = p_hipe + [-lcmf*sin(th_32);lcmf*cos(th_32)]; %swing thigh CoM
p_knee2e = p_hipe + [-Lf*sin(th_32);Lf*cos(th_32)]; %swing knee
pcmt2e = p_knee2e + [-lcmt*sin(th_42);lcmt*cos(th_42)]; %swing shin CoM
p_toe2e = p_knee2e + [-Lt*sin(th_42);Lt*cos(th_42)]; %swing leg-end

% Compute Je (the Jacobian at the swing leg end)
Je = jacobian(p_toe2e,qe); Je = simplify(Je);

%Calculate Velocities
Jcmt1e = jacobian(pcmt1e,qe); Jcmt1e = simplify(Jcmt1e);  vcmt1e = Jcmt1e*dqe;   vcmt1e = simplify(vcmt1e);
Jcmf1e = jacobian(pcmf1e,qe); Jcmf1e = simplify(Jcmf1e);  vcmf1e = Jcmf1e*dqe;   vcmf1e = simplify(vcmf1e);
JcmTe = jacobian(pcmTe,qe);   JcmTe = simplify(JcmTe);    vcmTe = JcmTe*dqe;      vcmTe = simplify(vcmTe);
Jcmf2e = jacobian(pcmf2e,qe); Jcmf2e = simplify(Jcmf2e);  vcmf2e = Jcmf2e*dqe;   vcmf2e = simplify(vcmf2e);
Jcmt2e = jacobian(pcmt2e,qe); Jcmt2e = simplify(Jcmt2e);  vcmt2e = Jcmt2e*dqe;   vcmt2e = simplify(vcmt2e);

%Calculate Kinetic Energy
Ke = 1/2*mt*transpose(vcmt1e)*vcmt1e+1/2*Jt*dth_41^2 + ... 
    1/2*mf*transpose(vcmf1e)*vcmf1e+1/2*Jf*dth_31^2 + ...
    1/2*mT*transpose(vcmTe)*vcmTe+1/2*JT*dth_1^2 + ...
    1/2*mf*transpose(vcmf2e)*vcmf2e+1/2*Jf*dth_32^2 + ... 
    1/2*mt*transpose(vcmt2e)*vcmt2e+1/2*Jt*dth_42^2;
Ke = simplify(Ke);

%Find Extended Mass-Inertia Matrix
dKe_dqdot = jacobian(Ke,dqe);
De        = jacobian(dKe_dqdot,dqe);
De        = simplify(De);

%Find extended Gravitational Terms Matrix
Ve = g*(mt*pcmt1e(2)+mf*pcmf1e(2)+mT*pcmTe(2)+mt*pcmt2e(2)+mf*pcmf2e(2));
Ve = simplify(V); %Calculate Potential Energy
Ge = jacobian(Ve,qe).';

%Find extended Coriolis/Centrifugal Terms Matrix
Ce = 0*De;
N = length(qe);
for k=1:N
    for j=1:N
        sum = 0*g;
        for i=1:N
            sum = sum + 1/2*(diff(De(k,j),qe(i)) + diff(De(k,i),qe(j)) - diff(De(i,j),qe(k)))*dqe(i);
        end
        Ce(k,j) = sum;
    end
end
Cedq = Ce*dqe;
Cedq = simplify(Cedq);

%Extended Input Matrix
Be = jacobian(qe([1:4]),qe).'; 
%% Write functions to .m files
%Output the matrices needed to solve the EoM
matlabFunction(D, 'vars', {q,robot_params}, 'file', 'inertia_Matrix', 'Optimize', false);
matlabFunction(Cdq, 'vars', {q,dq,robot_params}, 'file', 'Coriolis_Centrifugal', 'Optimize', false);
matlabFunction(G, 'vars', {q,robot_params}, 'file', 'gravity_terms', 'Optimize', false);
matlabFunction(B, 'vars', {q,robot_params}, 'file', 'input_Matrix', 'Optimize', false);
%ForwardKinematics: Outputs important position coordinates
matlabFunction(task_pos, 'vars', {q,robot_params}, 'file', 'forwardKinematics', 'Optimize', false);
%Output velocities of leg components
matlabFunction(task_vel, 'vars', {q,dq,robot_params}, 'file', 'taskVel', 'Optimize', false);

%Output Matrices needed to solve the Impact Dynamics
matlabFunction(De, 'vars', {qe,robot_params}, 'file', 'extended_inertia_Matrix', 'Optimize', false);
matlabFunction(Je, 'vars', {qe,robot_params}, 'file', 'extended_swing_Jacobian', 'Optimize', false);
matlabFunction(Cedq, 'vars', {qe,dqe,robot_params}, 'file', 'extended_Coriolis_Centrifugal', 'Optimize', false);
matlabFunction(Ge, 'vars', {qe,robot_params}, 'file', 'extended_gravity_terms', 'Optimize', false);
matlabFunction(Be, 'vars', {qe,robot_params}, 'file', 'extended_input_Matrix', 'Optimize', false);

%Eventually will use He = Cedq + Ge;

%% Write functions to .mex files
%codegen inertiaMatrix -args {zeros(5,1),zeros(13,1)}   -o Cfcn_inertiaMatrix -report
%codegen coriolisMatrix -args {zeros(5,1),zeros(5,1),zeros(13,1)}   -o Cfcn_coriolisMatrix -report
%codegen gravityMatrix -args {zeros(5,1),zeros(13,1)}   -o Cfcn_gravityMatrix -report
%codegen inputMatrix -args {zeros(5,1),zeros(13,1)}   -o Cfcn_inputMatrix -report
%codegen forwardKinematics -args {zeros(5,1),zeros(6,1)}   -o Cfcn_forwardKinematics -report
%codegen extended_inertia_Matrix -args {zeros(7,1),zeros(13,1)}   -o Cfcn_extended_inertia_Matrix -report
%codegen extended_swing_Jacobian -args {zeros(7,1),zeros(13,1)}   -o Cfcn_extended_swing_Jacobian -report
%% Test forward kinematics function (Sanity Check)
% close all
% %Dummy values to test forward kinematics
% %q = [q31  q32  q41  q42  q1].';
% Q = [205; 160; 20; 25; 3]; Q = deg2rad(Q);
% %linkLengths = [Lt lcmt LT lcmT Lf lcmf].';
% LL = [1;1;1;1;1;1];
% LI = [1;1;1;1;1;1;9.81];
% Robot_Params = [LL;LI];
% %x = [p_toe1;p_knee1;p_hip;pcmT;p_knee2;p_toe2];
% X = forwardKinematics(Q,Robot_Params);
% %Create a stick-figure plot showing the robot configuration
% hold on
% plot([X(1) X(3)],[X(2) X(4)],'g',[X(3) X(5)],[X(4) X(6)],'r', ...%toe>knee,knee>hip
% [X(5) X(7)],[X(6) X(8)],'b', ... %hip>torsoCoM
% [X(5) X(9)],[X(6) X(10)],'r:', ... %hip>knee
% [X(9) X(11)],[X(10) X(12)],'g:','LineWidth',2) %knee>toe
% legend('stance shin','stance thigh','torso','swing thigh','swing shin')
% axis equal
% grid on
% title('Robot Configuration (From Forward Kinematics)')
% xlabel('X (m)')
% ylabel('Y (m)')