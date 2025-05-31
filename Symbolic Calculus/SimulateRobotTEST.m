%% Simulate the robot
function [t,x]=SimulateRobotTEST(x0,dt,tf,robot_params)

%robot_params = [Lt lcmt LT lcmT Lf lcmf mt Jt mT JT mf Jf g].';

Lt = robot_params(1); lcmt = robot_params(2);  LT = robot_params(3); lcmT = robot_params(4); 
Lf = robot_params(5); lcmf = robot_params(6); mt = robot_params(7); Jt = robot_params(8);
mT = robot_params(9); JT = robot_params(10); mf = robot_params(11); Jf = robot_params(12);
g  = robot_params(13); 

t0 = 0;
t = t0:dt:tf;

%Use ODE45 solve ODE's numerically:
[t,x] = ode45(@func,t,x0);
    function xdot=func(t,x)
        %Calculate dynamic terms
        D = inertia_Matrix(x(1:5),robot_params);
        Cdq = Coriolis_Centrifugal(x(1:5),x(6:10),robot_params);
        G = gravity_terms(x(1:5),robot_params);
        H = Cdq + G;
        xdot = [x(6);x(7);x(8);x(9);x(10); -inv(D)*H];
    end
end 