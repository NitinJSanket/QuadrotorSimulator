function path = CalcReconsPath(Prev, goalID)
path = [goalID];
TempID = Prev(goalID);
while(TempID>0)
    path = [TempID; path];
    TempID = Prev(TempID);
end
path = path(path>0);
end