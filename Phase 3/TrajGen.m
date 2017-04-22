clc
clear all
close all

%% Define parameters
m = 2;
n = 3;

% Compute Spline coefficient matrix T, the problem is TX = B
T = ComputeT(m, n);

% StartConstraints and GoalConstraints are each a nx1 vector of constraints,
% position and upto (n-1)th derivative
StartConstraints = zeros(n,1);
GoalConstraints = zeros(n,1);
StartConstraints(1) = -5; % Start Position
GoalConstraints(1) = 5; % Goal Position

% This is a (m-2)x1 vector of positions
PositionConstraints = linspace(StartConstraints(1), GoalConstraints(1), m);
PositionConstraints = PositionConstraints(2:end-1)';

% Compute B matrix
B = ComputeB(m, n, StartConstraints, GoalConstraints, PositionConstraints);

% Compute X by solving TX = B
% Values of T are computed by some optimization technique
% For now, assume that time between time segments is 1.25*sqrt(sqrt(Dist))
AllPos = [StartConstraints(1); PositionConstraints; GoalConstraints(1)];
times = [0; diff(AllPos)];
times = sqrt(times);
times = cumsum(times);
t = sym('t', [m,1]);

% Substitute value of t in T
% Switch cases are needed to reduce substitutions or else the code is super
% slow
for count = 1:m
    switch count
        case 1
            T(1:n,1:2*n) =  subs(T(1:n,1:2*n), t(count), times(count));
        case m
            T(end-n+1:end, end-2*n+1:end) =  subs(T(end-n+1:end, end-2*n+1:end), t(count), times(count));
        otherwise
            T(2*n*(count-2)+n+1:2*n*(count-1)+n, 2*n*(count-2)+1:2*n*(count)) =...
                subs(T(2*n*(count-2)+n+1:2*n*(count-1)+n, 2*n*(count-2)+1:2*n*(count)), t(count), times(count));
    end
    
end

T = double(T);
% Solve for B by solving linear system of equations using
X = T\B;

% Plot outputs
Ts = linspace(0, times(end), 100);
for count = 1:length(Ts)
    Pos(count) = X'*(Ts(count).^(5:-1:0))';
end
plot(Ts, Pos);
