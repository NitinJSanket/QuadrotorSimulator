function Nbs = findNbs(CurrNode, sizeX, sizeY, sizeZ)
Nbs = [CurrNode+1; CurrNode-1; CurrNode + sizeY; CurrNode - sizeY; CurrNode + sizeY*sizeX; CurrNode - sizeY*sizeX];
% Exclude Nbs below zero and greater than limit size
Nbs = Nbs(Nbs>0 & Nbs<sizeZ*sizeY*sizeX);

end
