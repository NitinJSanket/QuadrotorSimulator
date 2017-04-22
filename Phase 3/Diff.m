% Differentiate the polynomial x n times and stack them
% Each row is a polynomial in t and n is a number
% NTerms is the number of derivatives you need
% xdiff is the stacked symbolic diff
function xdiff = Diff(t, n, NTerms)
if(nargin<3)
    NTerms = n-1;
end
% Permutation formula for differentiation
P = @(n, r)factorial(2*n-1:-1:r)./factorial(2*n-1-r:-1:0);
xdiff = sym(zeros(n-1,2*n));
for r = 1:NTerms
    xdiff(r,1:2*n-r) = P(n, r).*(t.^(2*n-1-r:-1:0));
end
end
