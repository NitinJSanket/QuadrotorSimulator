function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a CalcH if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
end
astar = false; % Always do Dijkstra

%% START OR GOAL HAS COLLISION
StartC = collide(map, start);
GoalC = collide(map, goal);
if(StartC||GoalC)
    path = [];
    return;
end

%%
startOrg = start;
goalOrg = goal;
path = []; % Path from source to dest
num_expanded = 0;
cost = inf; % Dist. from source to dest
xy_res = map(end, 1);
z_res = map(end, 3);
x0 = map(end-1, 1);
y0 = map(end-1, 2);
z0 = map(end-1, 3);
x1 = map(end-1, 4);
y1 = map(end-1, 5);
z1 = map(end-1, 6);

hold on;
%% Generate all "VALID" nodes
if (z1-z0)<z_res
    z_res = z1-z0;
end

AllPts = GetAllLocs([x0, y0, z0], [x1, y1, z1], xy_res, z_res);
AllXPos = (x0:xy_res:x1)';
AllYPos = (y0:xy_res:y1)';
AllZPos = (z0:z_res:z1)';

C = collide(map, AllPts);
if mod(goal(:,1),xy_res)
    goal(:,1) = goal(:,1) - mod(goal(:,1),xy_res);
end
if mod(goal(:,2),xy_res)
    goal(:,2) = goal(:,2) - mod(goal(:,2),xy_res);
end
if mod(goal(:,3),z_res)
    goal(:,3) = goal(:,3) - mod(goal(:,3),z_res);
end
if mod(start(:,1),xy_res)
    start(:,1) = start(:,1) - mod(start(:,1),xy_res);
end
if mod(start(:,2),xy_res)
    start(:,2) = start(:,2) - mod(start(:,2),xy_res);
end
if mod(start(:,3),z_res)
    start(:,3) = start(:,3) - mod(start(:,3),z_res);
end
% if z0==z1
%     start(:,3) = z0;
%     goal(:,3) = z0;
% end

NumNodes = size(AllPts,1);
Nodes = (1:NumNodes)';
StartNode = Nodes(ismember(AllPts,start,'rows'));
GoalNode = Nodes(ismember(AllPts,goal,'rows'));
NodesClosed = zeros(NumNodes,1);
NodesOpen = zeros(NumNodes,1);
FScore(1:NumNodes,1) = Inf;
FScore(StartNode) = 0;
NodesOpen(StartNode) = 1;

Prev = zeros(NumNodes,1);
GScore = zeros(NumNodes,1);
NodeCounter = 1;
FScore(StartNode,1) = GScore(StartNode,1) + CalcH(StartNode,GoalNode,AllPts,astar);
while ~isempty(NodesOpen)
    [~,CurrNode] = min(FScore);
    if (CurrNode==GoalNode)
        path = CalcReconsPath(Prev,CurrNode);
        break;
    end
    Nbs = findNbs(CurrNode,size(AllXPos,1),size(AllYPos,1),size(AllZPos,1));
    Nbs = Nbs(~C(Nbs));
    if isempty(Nbs)
        continue;
    end
    NodesClosed(CurrNode) = 1;
    NodesOpen(CurrNode) = NaN;
    NodeCounter = NodeCounter + 1;
    for i=1:size(Nbs)
        NbsIdx = Nbs(i);
        if NodesClosed(NbsIdx)==1;
            FScore(NbsIdx,:) = NaN;
            continue;
        end
        TempGScore = sqrt(sum(bsxfun(@minus, AllPts(CurrNode, :), AllPts(StartNode,:)).^2, 2));%sum(abs(bsxfun(@minus, AllPts(CurrNode, :), AllPts(StartNode,:))), 2);%sqrt(sum(bsxfun(@minus, AllPts(CurrNode, :), AllPts(StartNode,:)).^2, 2));% + CalcH(CurrNode,NbsIdx,AllPts,astar);
        b = NodesOpen(NbsIdx);
            if ~b||TempGScore<pdist2(AllPts(NbsIdx,:),AllPts(StartNode,:))
                Prev(NbsIdx) = CurrNode;
                GScore(NbsIdx) = TempGScore;
                FScore(NbsIdx)= GScore(NbsIdx) + CalcH(NbsIdx,GoalNode,AllPts,astar);
                if FScore(NbsIdx,:) == 0
                    FScore(NbsIdx,:) = NaN;
                end
                if ~b
                    NodesOpen(NbsIdx) = 1;
                end
            else
                FScore(NbsIdx) = NaN;
            end
    end
    FScore(CurrNode) = NaN;
end
path = AllPts(path,:);
path(1,:) = startOrg;
path(end,:) = goalOrg;
num_expanded = size(NodesClosed,1);
plot_path(map,path);
end


