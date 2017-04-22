%% Code by:
% Nitin J. Sanket (nitinsan@terpmail.umd.edu) and Chahat Deep Singh (chahat@umd.edu)
% PhD in Computer Science Student and Masters in Robotics Student at University of Maryland,
% College Park
% Function fits N^th order spline matrix T by minimizing the functional Integral
% (xn)^2, xn is the nth derivative of x
% N = 2*n-1
% The problem is TX = B, where T is the coefficient matrix, X is the spline
% coefficient vector and B is the constraints' vector
% Inputs:
% n: Specifies derivative number of minimization problem, solution will be
% 2n degree polynomial
% m: Number of waypoints including start and goal position
% Sample Usage:
% T = ComputeT(2, 3); should return a quintic polynomial matrix T
% Use this function along with: ComputeB

function T = ComputeT(m, n)
if(nargin<2)
    % Minimization problem is Integral of (xn)^2, nth derivative
    % this gives a (2n-1)th degree polynomial
    n = 3;
    % Number of waypoints including start and end points
    m = 2;
end

T = sym(zeros(2*n*(m-1)));
t = sym('t', [m,1]);

%% Start Point has n constraints, 1 position constraint and (n-1) derivative
% constrains
Pos1 = Post(t(1), n);
% Compute all differentials
Diff1 = Diff(t(1), n);

% First n x 2n matrix
T(1:n, 1:2*n) = [Pos1; Diff1];

%% End Point has n constraints, 1 position constraint and (n-1) derivative
% constrains
Posend = Post(t(end), n);
% Compute all differentials
Diffend = Diff(t(end), n);

% Last n x 2n matrix
T(end-n+1:end, end-2*n+1:end) = [Posend; Diffend];

%% Constraints between points
TAll = ConstBetweenPts(t, n, m);

%% Concatenate whole matrix
T(n+1:end-n, 1:end) = TAll;

end