function [desired_state] = smallStep(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

% There are 4 line segments in this "diamond helix"


if (t==0)
pos = [0 0 0]';
vel = [0 0 0]';
acc = [0 0 0]';
yaw = 0;
yawdot = 0;
else
    pos = [5 0 0]';
vel = [0 0 0]';
acc = [0 0 0]';
yaw = 0;
yawdot = 0;
end
    

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
