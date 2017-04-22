% Add additional initialization if you want to.
% You can use this space to pre-compute the trajectory to avoid
% repeatedly computing the same trajectory in every call of the
% "trajectory_generator" function

% Generate trajectory
disp('Generating Trajectory ...');
npoly = 3; % 2n-1 is the degree of the polynomial being fitted
% trajectory_generator([], [], map, path);

%% Dummy Map with Circle Trajectory
map = map(end-1:end,:);
NPts = 2;
x = linspace(-2, 2, NPts)';
y = linspace(-2, 2, NPts)';
z = linspace(-2, 2, NPts)';
path = [x,y,z];
NewTrajGen([], [], map, {path}, npoly)