clc
clear all
close all


% r = 5;
% NTurns = 1;
% MaxAng = NTurns*2*pi; % Number of turns in helix
% c = 2.5/MaxAng; % Determines end z point
% omega = 0.6; % rad/s
% t = 0:0.01:15;
% MaxTime = MaxAng/omega;
% a3 = -2*MaxAng/(omega*MaxTime^3);
% a2 = -(3/2)*a3*MaxTime;
% theta = omega*(a3*t.^3+a2*t.^2);
% x = r*cos(theta);
% y = r*sin(theta);
% z = c*theta;
% 
% plot3(x,y,z,'r*');

syms T x xdot f a3 a2 a1 a0 t real y z
%   y = sqrt(2)*t/T;
%   z = sqrt(2)*t/T;
f = a3*t^3+a2*t^2+a1*t+a0;
ti = T/4;
tf = T/2;
yi = 0.25;
yf = 0.5;
% xdot = diff(x, t);
% xi = subs(x, 0); 
% xf = subs(x, 4*T);
% xidot = subs(xdot, 0);
% xfdot = subs(xdot, 0);
v = ([1 ti ti^2 ti^3; ...
     0 1 2*ti 3*ti^2; ...
     1 tf tf^2 tf^3; ...
     0 1 2*tf 3*tf^2;])\[yi 0 yf 0]'; 
%  a3 = v(1);
%  a2 = v(2);
%  a1 = v(3);
%  a0 = v(4);
a = subs(f, a0, v(1));
a = subs(a, a1, v(2));
a = subs(a, a2, v(3));
a = subs(a, a3, v(4));
% disp(a);
f = a;
y = f; % Linearly increase x from 0 to 1
% x = subs(x, f);
ydot = diff(y, t);
yddot = diff(ydot, t);
disp(y);
disp(ydot);
disp(yddot);

T = 10;
t = double(subs(ti):0.01:subs(tf));
ynum = double(subs(y));
plot(ynum);