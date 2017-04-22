function [desired_state] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path0 into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path0) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path0: This is the path0 returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path0
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

persistent path0 AllWayPts map0 MinRes
Vavg = 0.9; % m/s
NumDiv = 100;
yaw = 0;
yawdot = 0;
x = 0;
y = 0;
z = 0;
xdot = 0;
ydot = 0;
zdot = 0;
xddot = 0;
yddot = 0;
zddot = 0;
if nargin==4
    path0 = path{1};
    map0 = map;
    xy_res = map0(end, 1);
    z_res = map0(end, 3);
    min_res = min(xy_res, z_res);
    x0 = map0(end-1, 1);
    y0 = map0(end-1, 2);
    z0 = map0(end-1, 3);
    x1 = map0(end-1, 4);
    y1 = map0(end-1, 5);
    z1 = map0(end-1, 6);
    NumWayPts = size(path0, 1);
    MinRes = min(xy_res, z_res);
    
    %% Try to Shorten the path
    disp('Shortening Path....');
    n = [0:MinRes/NumDiv:1]'; % Variation in parameter
    n = repmat(n, 1, 3);
    CurrPos = 1;
    GoalPos = NumWayPts;
    WayPtsIdxs(1) = CurrPos;
    NextPos = GoalPos;
    while(CurrPos~=GoalPos)
        % Start from goal position and backtrack
        % Vector pointing in direction indicated by CurrPos and NextPos
        CurrVec = repmat(path0(CurrPos, :), size(n, 1), 1) + n.*(repmat(path0(NextPos, :), size(n, 1), 1) - repmat(path0(CurrPos, :), size(n,1), 1));
        C = collide(map0, CurrVec); % Check if vector collides
        %     plot3(CurrVec(:, 1),CurrVec(:, 2),CurrVec(:, 3), 'g');
        %     pause(0.01);
        if(~any(C)) % If no collision
            WayPtsIdxs(end+1) = NextPos;
            CurrPos = NextPos;
            NextPos = GoalPos;
%             disp(CurrPos);
            %         plot3(CurrVec(:, 1),CurrVec(:, 2),CurrVec(:, 3), 'b');
            %         pause(0.5);
        else
            NextPos = NextPos-1;
            if(NextPos==WayPtsIdxs(end))
                disp('CANT FIND A PATH!');
                break;
            else
                continue;
            end
        end
    end
    
    AllWayPts(:, 1:3) = path0(WayPtsIdxs, :); 
    plot3(AllWayPts(:,1), AllWayPts(:,2), AllWayPts(:,3), 'b');
    %     figure,
    %     plot3(AllWayPts(:,1), AllWayPts(:,2), AllWayPts(:,3), 'b.');
    
    AllWayPts(1, 4) = 0;
    for i = 2:size(AllWayPts,1)
        AllWayPts(i, 4) =  1.25.*sqrt(sqrt(sum((AllWayPts(i, 1:3)-AllWayPts(i-1, 1:3)).^2))) + AllWayPts(i-1, 4);
        % Fitting for x co-ordinate
        q0 = AllWayPts(i-1, 1);
        v0 = 0;
        a0 = 0;
        q1 = AllWayPts(i, 1);
        v1 = 0;
        a1 = 0;
        t0 = AllWayPts(i-1, 4);
        tf = AllWayPts(i, 4);
        Conditions = [q0 v0 a0 q1 v1 a1]';
        Coeff = FitQuintic(t0, tf, Conditions);
        AllWayPts(i, 5:10) = Coeff;
        % Fitting for y co-ordinate
        q0 = AllWayPts(i-1, 2);
        v0 = 0;
        a0 = 0;
        q1 = AllWayPts(i, 2);
        v1 = 0;
        a1 = 0;
        t0 = AllWayPts(i-1, 4);
        tf = AllWayPts(i, 4);
        Conditions = [q0 v0 a0 q1 v1 a1]';
        Coeff = FitQuintic(t0, tf, Conditions);
        AllWayPts(i, 11:16) = Coeff;
        % Fitting for z co-ordinate
        q0 = AllWayPts(i-1, 3);
        v0 = 0;
        a0 = 0;
        q1 = AllWayPts(i, 3);
        v1 = 0;
        a1 = 0;
        t0 = AllWayPts(i-1, 4);
        tf = AllWayPts(i, 4);
        Conditions = [q0 v0 a0 q1 v1 a1]';
        Coeff = FitQuintic(t0, tf, Conditions);
        AllWayPts(i, 17:22) = Coeff;
    end
else
    for i = 2:size(AllWayPts, 1)
        if(t<=AllWayPts(i,4))&&(t>AllWayPts(i-1,4))
            x = AllWayPts(i, 5) + AllWayPts(i, 6)*t + AllWayPts(i, 7)*t^2 +...
                AllWayPts(i, 8)*t^3 + AllWayPts(i, 9)*t^4 + AllWayPts(i, 10)*t^5;
            xdot = AllWayPts(i, 6) + AllWayPts(i, 7)*2*t + AllWayPts(i, 8)*3*t^2 +...
                AllWayPts(i, 9)*4*t^3 + AllWayPts(i, 10)*5*t^4;
            xddot = AllWayPts(i, 7)*2 + AllWayPts(i, 8)*6*t + AllWayPts(i, 9)*12*t^2 +...
                AllWayPts(i, 10)*20*t^3;
            
            y = AllWayPts(i, 11) + AllWayPts(i, 12)*t + AllWayPts(i, 13)*t^2 +...
                AllWayPts(i, 14)*t^3 + AllWayPts(i, 15)*t^4 + AllWayPts(i, 16)*t^5;
            ydot = AllWayPts(i, 12) + AllWayPts(i, 13)*2*t + AllWayPts(i, 14)*3*t^2 +...
                AllWayPts(i, 15)*4*t^3 + AllWayPts(i, 16)*5*t^4;
            yddot = AllWayPts(i, 13)*2 + AllWayPts(i, 14)*6*t + AllWayPts(i, 15)*12*t^2 +...
                AllWayPts(i, 16)*20*t^3;
            
            z = AllWayPts(i, 17) + AllWayPts(i, 18)*t + AllWayPts(i, 19)*t^2 +...
                AllWayPts(i, 20)*t^3 + AllWayPts(i, 21)*t^4 + AllWayPts(i, 22)*t^5;
            zdot = AllWayPts(i, 18) + AllWayPts(i, 19)*2*t + AllWayPts(i, 20)*3*t^2 +...
                AllWayPts(i, 21)*4*t^3 + AllWayPts(i, 22)*5*t^4;
            zddot = AllWayPts(i, 19)*2 + AllWayPts(i, 20)*6*t + AllWayPts(i, 21)*12*t^2 +...
                AllWayPts(i, 22)*20*t^3;
            
            
            pos = [x;y;z];
            vel = [xdot;ydot;zdot];
            acc = [xddot;yddot;zddot];
            
            desired_state.pos = pos(:);
            desired_state.vel = vel(:);
            desired_state.acc = acc(:);
            desired_state.yaw = yaw;
            desired_state.yawdot = yawdot;
        elseif t>AllWayPts(end,4)
            x = AllWayPts(end, 1);
            y = AllWayPts(end, 2);
            z = AllWayPts(end, 3);
            xdot = 0;
            ydot = 0;
            zdot = 0;
            xddot = 0;
            yddot = 0;
            zddot = 0;
            pos = [x;y;z];
            vel = [xdot;ydot;zdot];
            acc = [xddot;yddot;zddot];
            
            desired_state.pos = pos(:);
            desired_state.vel = vel(:);
            desired_state.acc = acc(:);
            desired_state.yaw = yaw;
            desired_state.yawdot = yawdot;
        end
        
    end
end
pos = [x;y;z];
vel = [xdot;ydot;zdot];
acc = [xddot;yddot;zddot];
desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;
end
