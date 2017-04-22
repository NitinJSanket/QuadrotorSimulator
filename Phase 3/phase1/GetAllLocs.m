function AllLocs = GetAllLocs(in, out, xy_res, z_res)
x = in(:,1):xy_res:out(:,1);
y = in(:,2):xy_res:out(:,2);
if(z_res==0)
    z = in(:, 3);
else
z = in(:,3):z_res:out(:,3);
end
[x, y, z] = meshgrid(x, y, z);
AllLocs = [x(:), y(:), z(:)];
end
