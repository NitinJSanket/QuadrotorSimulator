function [desired_state] = NewTrajGen(t, qn, map, path, npoly)
% NewTrajGen: Turn a Dijkstra or A* path0 into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% NewTrajGen([], [], map, path0, []) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path0: This is the path0 returned by your planner (dijkstra function),
% size is m x 3
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path0
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

% TODO: This code assumes that collision checking has already been done
% before.

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

persistent path0 AllWayPts map0 m n
if nargin==5
    path0 = path{1};
    m = size(path0, 1);
    n = npoly;
    % Compute Spline coefficient matrix T, the problem is TX = B
    T = ComputeT(m, n);
    yaw = 0;
    yawdot = 0;
    % Assume start derivative constraints as zero
    % Each column is a constraint for 1 dimension
    PosDots = zeros(n-1,3);
    
    % Compute B matrix for x, y and z separately
    % Compute X by solving TX = B
    % Values of T are computed by some optimization technique
    % For now, assume that time between time segments is sqrt(Dist)
    t = sym('t', [m,1]);
    
    times = [0; sqrt(sum(diff(path0,1).^2,2))];
    times = cumsum(times);
    
    for numdims = 1:3
        B{numdims} = ComputeB(m, n, [path0(1,numdims); PosDots(:,numdims)]', [path0(end,numdims); PosDots(:,numdims)]', path0(1:end-2,numdims));
        TTemp{numdims} = T;
        % Substitute value of t in T
        % Switch cases are needed to reduce substitutions or else the code is super
        % slow
        for count = 1:m
            switch count
                case 1
                    TTemp{numdims}(1:n,1:2*n) =...
                        subs(TTemp{numdims}(1:n,1:2*n), t(count), times(count));
                case m
                    TTemp{numdims}(end-n+1:end, end-2*n+1:end) =...
                        subs(TTemp{numdims}(end-n+1:end, end-2*n+1:end), t(count), times(count));
                otherwise
                    TTemp{numdims}(2*n*(count-2)+n+1:2*n*(count-1)+n, 2*n*(count-2)+1:2*n*(count)) =...
                        subs(TTemp{numdims}(2*n*(count-2)+n+1:2*n*(count-1)+n, 2*n*(count-2)+1:2*n*(count)), t(count), times(count));
            end
            
        end
        TTemp{numdims} = double(TTemp{numdims});
        % Solve for B by solving linear system of equations using
        SplineCoeff{numdims} = TTemp{numdims}\B{numdims};
    end
    AllWayPts = zeros(m, 4+6*n);
    AllWayPts(:, 1:3) = path0;
    AllWayPts(:, 4) = times;
    for numdims = 1:3
        AllWayPts(2:end, 2*n*(numdims-1)+5:2*n*(numdims)+4) = reshape(SplineCoeff{numdims}, m-1, 2*n);
    end
else
    for step = 2:size(AllWayPts, 1)
        P = @(n, r)factorial(2*n-1:-1:r)./factorial(2*n-1-r:-1:0);
        if(t<=AllWayPts(step,4))&&(t>AllWayPts(step-1,4)|| step == 2)
            % Compute x dimension
            x = AllWayPts(step, 5:2*n+4)*(t.^(2*n-1:-1:0))';
            
            xdot = (padarray(P(n,1), [0,1],'post').*AllWayPts(step,5:2*n+4))*(t.^(2*n-1:-1:0))';
            
            xddot = (padarray(P(n,2), [0,2],'post').*AllWayPts(step, 5:2*n+4))*(t.^(2*n-1:-1:0))';
            
            % Compute y dimension
            y = AllWayPts(step, 2*n+5:4*n+4)*(t.^(2*n-1:-1:0))';
            
            ydot = (padarray(P(n,1), [0,1],'post').*AllWayPts(step, 2*n+5:4*n+4))*(t.^(2*n-1:-1:0))';
            
            yddot = (padarray(P(n,2), [0,2],'post').*AllWayPts(step, 2*n+5:4*n+4))*(t.^(2*n-1:-1:0))';
            
            
            % Compute z dimension
            z = AllWayPts(step, 4*n+5:6*n+4)*(t.^(2*n-1:-1:0))';
            
            zdot = (padarray(P(n,1), [0,1],'post').*AllWayPts(step, 4*n+5:6*n+4))*(t.^(2*n-1:-1:0))';
            
            zddot = (padarray(P(n,2), [0,2],'post').*AllWayPts(step, 4*n+5:6*n+4))*(t.^(2*n-1:-1:0))';
            
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
