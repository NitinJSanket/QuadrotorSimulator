function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================
kp = [4.8 4.8 50]'; % X Y Z 50 16 good for Z
kd = [6.9 6.9 100]'; % X Y Z
kpAng = [0.1 0.1 1]'; % Roll Pitch Yaw
kdAng = [0.02 0.02 1]'; % Roll Pitch Yaw
phi = qd{qn}.euler(1);
theta = qd{qn}.euler(2);
psi = qd{qn}.euler(3);
% rddotcmd = qd{qn}.acc_des + kd'*(qd{qn}.vel_des-qd{qn}.vel)+kp'*(qd{qn}.pos_des-qd{qn}.pos);
rddotcmd1 = qd{qn}.acc_des(1) + kd(1)*(qd{qn}.vel_des(1)-qd{qn}.vel(1))+kp(1)*(qd{qn}.pos_des(1)-qd{qn}.pos(1));
rddotcmd2 = qd{qn}.acc_des(2) + kd(2)*(qd{qn}.vel_des(2)-qd{qn}.vel(2))+kp(2)*(qd{qn}.pos_des(2)-qd{qn}.pos(2));
rddotcmd3 = qd{qn}.acc_des(3) + kd(3)*(qd{qn}.vel_des(3)-qd{qn}.vel(3))+kp(3)*(qd{qn}.pos_des(3)-qd{qn}.pos(3));
u1 = params.mass*params.grav + params.mass*rddotcmd3;
phi_des = (1/params.grav)*(rddotcmd1*sin(qd{qn}.yaw_des)-rddotcmd2*cos(qd{qn}.yaw_des));
if(abs(phi_des)>pi*40/180)
    phi_des = sign(phi_des)*pi*40/180;
end
theta_des = (1/params.grav)*(rddotcmd1*cos(qd{qn}.yaw_des)+rddotcmd2*sin(qd{qn}.yaw_des));
if(abs(theta_des)>pi*40/180)
    theta_des = sign(theta_des)*pi*40/180;
end
p_des = 0;
q_des = 0;
% Desired roll, pitch and yaw
psi_des = qd{qn}.yaw_des;
r_des = qd{qn}.yawdot_des;
p = qd{qn}.omega(1);
q = qd{qn}.omega(2);
r = qd{qn}.omega(3);

u2 = [kpAng(1)*(phi_des-phi)+kdAng(1)*(p_des-p);...
      kpAng(2)*(theta_des-theta)+kdAng(2)*(q_des-q);...
      kpAng(3)*(psi_des-psi)+kdAng(3)*(r_des-r)];


% Thurst
F    = u1;

% Moment
M    = u2;
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
