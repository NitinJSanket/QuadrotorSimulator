function Nbs = findNbs(CurrNode, sizeX, sizeY, sizeZ, map, AllPts)
% xy_res = map(end, 1);
% z_res = map(end, 3);
x0 = map(end-1, 1);
y0 = map(end-1, 2);
z0 = map(end-1, 3);
x1 = map(end-1, 4);
y1 = map(end-1, 5);
z1 = map(end-1, 6);

% xa = CurrNode - 1;
% xb = CurrNode + 1;
% ya = CurrNode - sizeY;
% yb = CurrNode + sizeY;
% za = CurrNode - sizeX*sizeY;
% zb = CurrNode + sizeX*sizeY;
CurrNodePos = AllPts(CurrNode, :);
% if(CurrNodePos(1)<=x0) % X Boundary
%     Nbs = [CurrNode+1; CurrNode + sizeY; CurrNode - sizeY; CurrNode + sizeX*sizeY; CurrNode - sizeX*sizeY; ];
% elseif(CurrNodePos(1)>=x1)
%     Nbs = [CurrNode-1; CurrNode + sizeY; CurrNode - sizeY; CurrNode + sizeX*sizeY; CurrNode - sizeX*sizeY];
% elseif(CurrNodePos(2)<=y0)
%     Nbs = [CurrNode+1; CurrNode-1; CurrNode + sizeY; CurrNode + sizeX*sizeY; CurrNode - sizeX*sizeY];
% elseif(CurrNodePos(2)>=y1)
%     Nbs = [CurrNode+1; CurrNode-1; CurrNode - sizeY; CurrNode + sizeX*sizeY; CurrNode - sizeX*sizeY];
% elseif(CurrNodePos(3)<=z0)
%     Nbs = [CurrNode+1; CurrNode-1; CurrNode + sizeY; CurrNode - sizeY; CurrNode + sizeX*sizeY];
% elseif(CurrNodePos(3)>=z1)
%     Nbs = [CurrNode+1; CurrNode-1; CurrNode + sizeY; CurrNode + sizeX*sizeY; CurrNode - sizeX*sizeY];
% else
%     Nbs = [CurrNode+1; CurrNode-1; CurrNode + sizeY; CurrNode - sizeY; CurrNode + sizeX*sizeY; CurrNode - sizeX*sizeY];
% end
% Exclude Nbs below zero and greater than limit size
Nbs = [CurrNode-1; CurrNode+1; CurrNode - sizeY; CurrNode + sizeY; CurrNode - sizeX*sizeY; CurrNode + sizeX*sizeY];
if(CurrNodePos(1)<=x0)
    Nbs(3) = NaN;
end
if(CurrNodePos(1)>=x1)
    Nbs(4) = NaN;
end
if(CurrNodePos(2)<=y0)
    Nbs(1) = NaN;
end
if(CurrNodePos(2)>=y1)
    Nbs(2) = NaN;
end
if(CurrNodePos(3)<=z0)
    Nbs(5) = NaN;
end
if(CurrNodePos(3)>=z1)
    Nbs(6) = NaN;
end

Nbs = Nbs(~isnan(Nbs));
if(sizeZ==0)
    Nbs = Nbs(Nbs>0 & Nbs<sizeX*sizeY);
else
Nbs = Nbs(Nbs>0 & Nbs<sizeX*sizeY*sizeZ);
end
end
