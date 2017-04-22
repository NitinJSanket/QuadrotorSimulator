clc
clear all
close all
addpath(genpath('./'));

%% Load a path
% map has boundary as first row
% map has resolution as second row
map = [0, 0, 0, 20, 5, 6, 0, 0, 0;...
      0.1, 0.1, 0.5, 0, 0, 0, 0, 0, 0];
  
% load('path.mat');
NPts = 100;
Ang = 0:2*pi/(NPts-1):1.9*pi;
R = 10;
path = {[R*cos(Ang)', R*sin(Ang)', zeros(length(Ang),1)]};
start = {[R, 0, 0]};
stop  = {[R*cos(1.9*pi)', R*sin(1.9*pi)', 0]};
plot3(path{1}(:,1),path{1}(:,2),path{1}(:,3),'r.');
grid on;
axis equal;

%% Additional init script
init_script;

%% Run trajectory
% trajectory = test_trajectory(start, stop, map, path, true); % with visualization
