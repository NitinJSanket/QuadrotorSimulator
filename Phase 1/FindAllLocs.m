function AllPts = FindAllLocs(AllX, AllY, AllZ)
AllXOrg = numel(AllX);
AllYOrg = numel(AllY);
AllZOrg = numel(AllZ);

AllX = repmat(AllX',1,AllYOrg*AllZOrg)';
AllX = AllX(:);

AllY = repmat(AllY',1,AllZOrg)';
AllY = AllY(:)';
AllY = repmat(AllY,1,AllXOrg);
AllY = AllY';

AllZ = repmat(AllZ,1,AllXOrg*AllYOrg)';

AllPts = [AllX, AllY, AllZ];
end
