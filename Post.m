% Position at time t given t and n
function Post = Post(t, n)
Post = (t*ones(1,2*n)).^(2*n-1:-1:0);
end