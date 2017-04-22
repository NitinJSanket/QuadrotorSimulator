clc
clear all
close all

% r = 5;
% NTurns = 1;
% MaxAng = NTurns*2*pi;
% c = 2.5/MaxAng;
% StepSize = pi/100;
% MaxVel = 1; % m/s
% 
% t = 0:StepSize:MaxAng;
% x = r*cos(t);
% y = r*sin(t);
% z = c*t;

x = [0 0.25 0.5 0.75 1.0];
y = [0 sqrt(2) 0 -sqrt(2) 0];
z = [0 sqrt(2) 2*sqrt(2) sqrt(2) 0];
figure,
axis([-5 5 -5 5 -5 5]);
grid on;
hold on;
for i = 1:5
plot3(x(i),y(i),z(i), 'r*');
pause(0.5);
end

xlabel('x');
ylabel('y');
zlabel('z');

% Dist = MaxAng*sqrt(r^2+c^2);
% disp(['Distance to be travelled ', num2str(Dist), 'm']);
% T = Dist/MaxVel;

% a = 0.5;
% c = 5.0;
% t = pi/2:0.01:10*pi;
% 
% x = a*sin(t);
% y = a*cos(t);
% z = t/(2*pi*c);
% 
% figure(1)
% plot3(x, y, z); 
% xlabel('x'); ylabel('y'); title('Circula helix');