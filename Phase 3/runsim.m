clc
clear all
close all
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('maps/map3.txt', 0.1, 0.5, 0.25);
close all
% % start = {[0.0  -4.9 0.2]};
% % stop  = {[6.0  18.0-6 3.0]};
% start = {[1 3 5]};
% stop  = {[19 3 5]};
% nquad = length(start);
% for qn = 1:nquad
%     path{qn} = dijkstra(map, start{qn}, stop{qn}, 1);
% end
% if nquad == 1
%     plot_path(map, path{1});
% else
%     % you could modify your plot_path to handle cell input for multiple robots
% end

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory({[x(1), y(1), z(1)]}, {[x(end), y(end), z(end)]}, map, path, true); % with visualization
