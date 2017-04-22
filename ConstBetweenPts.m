function TAll = ConstBetweenPts(t, n, m)
% Generate constraints between points
TAll = sym(zeros(2*n*(m-2),2*n*(m-1)));
ttemp = sym('ttemp');
% Generate position constraints at m due to spline m-1 and m
Posm = Post(ttemp, n);
Posmplus1 = Post(ttemp, n);
% Generate differential constraints
Diffm = Diff(ttemp, n, 2*(n-1));
Diffmplus1 = -Diff(ttemp, n, 2*(n-1));
% Concatenate all the constraints at time m
Tm = sym(zeros(2*n,4*n));
Tm(1,1:2*n) = Posm;
Tm(2,2*n+1:4*n) = Posmplus1;
Tm(3:2*n,1:4*n) = [Diffm,Diffmplus1];

% Replace the ttemp by current time - this avoids recomputing derivatives
% everytime
for count = 1:m-2
TAll(2*n*(count-1)+1:2*n*count, 2*n*(count-1)+1:2*n*(count+1)) = subs(Tm, ttemp, t(count+1));
end
end