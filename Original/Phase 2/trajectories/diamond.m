function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

% There are 4 line segments in this "diamond helix"

T = 8.8; % Time for completion
x = (12*t^2)/T^2 - (32*t^3)/T^3;
xdot = (24*t)/T^2 - (96*t^2)/T^3;
xddot = 24/T^2 - (192*t)/T^3;
y = (48*2^(1/2)*t^2)/T^2 - (128*2^(1/2)*t^3)/T^3;
ydot = (96*2^(1/2)*t)/T^2 - (384*2^(1/2)*t^2)/T^3;
yddot = (96*2^(1/2))/T^2 - (768*2^(1/2)*t)/T^3;
z = (48*2^(1/2)*t^2)/T^2 - (128*2^(1/2)*t^3)/T^3;
zdot = (96*2^(1/2)*t)/T^2 - (384*2^(1/2)*t^2)/T^3;
zddot = (96*2^(1/2))/T^2 - (768*2^(1/2)*t)/T^3;

if(t>=0 && t<T/4)
x = (12*t^2)/T^2 - (32*t^3)/T^3;
xdot = (24*t)/T^2 - (96*t^2)/T^3;
xddot = 24/T^2 - (192*t)/T^3;
y = (48*2^(1/2)*t^2)/T^2 - (128*2^(1/2)*t^3)/T^3;
ydot = (96*2^(1/2)*t)/T^2 - (384*2^(1/2)*t^2)/T^3;
yddot = (96*2^(1/2))/T^2 - (768*2^(1/2)*t)/T^3;
z = (48*2^(1/2)*t^2)/T^2 - (128*2^(1/2)*t^3)/T^3;
zdot = (96*2^(1/2)*t)/T^2 - (384*2^(1/2)*t^2)/T^3;
zddot = (96*2^(1/2))/T^2 - (768*2^(1/2)*t)/T^3;
end

if(t>=T/4 && t<T/2)
x = (36*t^2)/T^2 - (12*t)/T - (32*t^3)/T^3 + 3/2;
xdot = (72*t)/T^2 - 12/T - (96*t^2)/T^3;
xddot = 72/T^2 - (192*t)/T^3;
y = (128*2^(1/2)*t^3)/T^3 - (144*2^(1/2)*t^2)/T^2 - 4*2^(1/2) + (48*2^(1/2)*t)/T;
ydot =  (48*2^(1/2))/T + (384*2^(1/2)*t^2)/T^3 - (288*2^(1/2)*t)/T^2;
yddot = (768*2^(1/2)*t)/T^3 - (288*2^(1/2))/T^2;
z = 6*2^(1/2) + (144*2^(1/2)*t^2)/T^2 - (128*2^(1/2)*t^3)/T^3 - (48*2^(1/2)*t)/T;
zdot = (288*2^(1/2)*t)/T^2 - (384*2^(1/2)*t^2)/T^3 - (48*2^(1/2))/T;
zddot = (288*2^(1/2))/T^2 - (768*2^(1/2)*t)/T^3;
end

if(t>=T/2 && t<3*T/4)
x = (60*t^2)/T^2 - (36*t)/T - (32*t^3)/T^3 + 15/2;
xdot = (120*t)/T^2 - 36/T - (96*t^2)/T^3;
xddot = 120/T^2 - (192*t)/T^3;
y = (128*2^(1/2)*t^3)/T^3 - (240*2^(1/2)*t^2)/T^2 - 28*2^(1/2) + (144*2^(1/2)*t)/T;
ydot = (144*2^(1/2))/T + (384*2^(1/2)*t^2)/T^3 - (480*2^(1/2)*t)/T^2;
yddot = (768*2^(1/2)*t)/T^3 - (480*2^(1/2))/T^2;
z = (128*2^(1/2)*t^3)/T^3 - (240*2^(1/2)*t^2)/T^2 - 26*2^(1/2) + (144*2^(1/2)*t)/T;
zdot = (144*2^(1/2))/T + (384*2^(1/2)*t^2)/T^3 - (480*2^(1/2)*t)/T^2;
zddot = (768*2^(1/2)*t)/T^3 - (480*2^(1/2))/T^2;
end

if(t>=3*T/4 && t<T)
x = (84*t^2)/T^2 - (72*t)/T - (32*t^3)/T^3 + 21;
xdot = (168*t)/T^2 - 72/T - (96*t^2)/T^3;
xddot = 168/T^2 - (192*t)/T^3;
y = 80*2^(1/2) + (336*2^(1/2)*t^2)/T^2 - (128*2^(1/2)*t^3)/T^3 - (288*2^(1/2)*t)/T;
ydot = (672*2^(1/2)*t)/T^2 - (384*2^(1/2)*t^2)/T^3 - (288*2^(1/2))/T;
yddot = (672*2^(1/2))/T^2 - (768*2^(1/2)*t)/T^3;
z = (128*2^(1/2)*t^3)/T^3 - (336*2^(1/2)*t^2)/T^2 - 80*2^(1/2) + (288*2^(1/2)*t)/T;
zdot = (288*2^(1/2))/T + (384*2^(1/2)*t^2)/T^3 - (672*2^(1/2)*t)/T^2;
zddot = (768*2^(1/2)*t)/T^3 - (672*2^(1/2))/T^2;
end

if (t>=T)
    x = 1;
    y = 0;
    z = 0;
    xdot = 0;
    ydot = 0;
    zdot = 0;
    xddot = 0;
    yddot = 0;
    zddot = 0;
end

pos = [x; y; z];
vel = [xdot; ydot; zdot];
acc = [xddot; yddot; zddot];
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end

