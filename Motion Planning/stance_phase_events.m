function [value,isterminal,direction] = stance_phase_events(t,x,leg,theta_plus,theta_minus,alpha,robot_params)

% This function calculates the evnt function for the stance phase
%
% Inputs:
% t: time
% x: 10-dim state variable during the stance phase
% leg: 0 for the right stance and 1 for the left stance
% theta_plus:  initial value of theta on the orbit
% theta_minus: final value of theta on the orbit
% alpha: coefficient matrix for the bezier polynomial
%
% Outputs:
% value: the event-function
% isterminal: stop condition
% direction: the time derivative sign
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Position and velocity
q  = x(1:5);
dq = x(6:10);

% Swing leg end
%disp(x)
task_pos = forwardKinematics(q,robot_params);
%disp(task_pos)
value = task_pos(2,6); %extract y-coordinate of swing leg-end

[s,ds,ds_dq] = scaled_phasing_variable(x,leg,theta_plus,theta_minus);
if s>0.9
%if task_pos(1,6) > 0 %if x-coordinate of swing leg-end > 0
    % nothing
    if value == 0
        %disp("event")
    end
else
    value = 1;
end

% Stop conditions
isterminal(1) = 1;
direction(1)  = -1; % It is decreasing

end
