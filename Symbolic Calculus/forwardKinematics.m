function task_pos = forwardKinematics(in1,in2)
%FORWARDKINEMATICS
%    TASK_POS = FORWARDKINEMATICS(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    02-Apr-2019 18:01:54

Lf = in2(5,:);
Lt = in2(1,:);
lcmT = in2(4,:);
q1 = in1(5,:);
q31 = in1(1,:);
q32 = in1(2,:);
q41 = in1(3,:);
q42 = in1(4,:);
task_pos = reshape([0.0,0.0,Lt.*sin(q1+q31-q41),-Lt.*cos(q1+q31-q41),Lt.*sin(q1+q31-q41)+Lf.*sin(q1+q31),-Lt.*cos(q1+q31-q41)-Lf.*cos(q1+q31),Lt.*sin(q1+q31-q41)+Lf.*sin(q1+q31)-lcmT.*sin(q1),-Lt.*cos(q1+q31-q41)-Lf.*cos(q1+q31)+lcmT.*cos(q1),Lt.*sin(q1+q31-q41)+Lf.*sin(q1+q31)-Lf.*sin(q1+q32),-Lt.*cos(q1+q31-q41)-Lf.*cos(q1+q31)+Lf.*cos(q1+q32),Lt.*sin(q1+q31-q41)-Lt.*sin(q1+q32-q42)+Lf.*sin(q1+q31)-Lf.*sin(q1+q32),-Lt.*cos(q1+q31-q41)+Lt.*cos(q1+q32-q42)-Lf.*cos(q1+q31)+Lf.*cos(q1+q32)],[2,6]);
