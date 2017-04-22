function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.
C = zeros(size(points, 1), 1);
if(size(map, 1)<=2)
    return;
end

for i = 1:size(points, 1)
    for j = 1:size(map, 1)-2
        % points is x,y,z and map is x0, y0, z0, x1, y1, z1
        if(points(i, 1)>=map(j, 1))&&(points(i, 1)<=map(j, 4))...
                &&(points(i, 2)>=map(j, 2))&&(points(i, 2)<=map(j, 5))...
                &&(points(i, 3)>=map(j, 3))&&(points(i, 3)<=map(j, 6))
            C(i) = 1;
            break;
        end
    end
end

end
