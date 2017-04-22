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
% StartConstraints and GoalConstraints are each a nx1 vector of constraints, 
% position and upto (n-1)th derivative
% This is a (m-2)x1 vector of positions 
% Use this function along with: ComputeT

function B = ComputeB(m, n, StartConstraints, GoalConstraints, PositionConstraints)
% Compute constraints' vector B
B = zeros(2*n*(m-1),1);
B(1:n) = StartConstraints;
B(n+1:2*n:end-n) = PositionConstraints;
B(end-n+1:end) = GoalConstraints; 
end
