function h = CalcH(point, GoalNode, AllPts, astar)
if astar
     h = sqrt(sum(bsxfun(@minus, AllPts(point, :), AllPts(GoalNode,:)).^2, 2));
%      h = sum(abs(bsxfun(@minus, AllPts(point, :), AllPts(GoalNode,:))));
else
    h = 0;
end
end