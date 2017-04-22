function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.
NumCellsMax = 20;
DiscretizationFactor = repmat([xy_res, xy_res, z_res], 1, 2)'; % xy_res, z_res
fid = fopen(filename);
CurrLine = cell(NumCellsMax, 1);
k = 1;
figure,
grid on;
hold on;
while (k<=NumCellsMax)
    Temp = fgetl(fid);
    if(Temp == -1)
        break;
    end
    if(numel(Temp))
        if (Temp(1)=='#') % Chuck the comments
            continue;
        end
    else
        continue;
    end
    CurrLine{k} = strsplit(Temp);
    if(size(CurrLine{k})) % Chuck empty lines in the data
        k = k+1;
    end
end

if(isempty(CurrLine{1}))
    map = zeros(0, 9);
    map(end+1, :) = nan;
    map(end+1, :) = [xy_res, xy_res, z_res, 0, 0, 0, 0, 0, 0];
    return;
end
% use fgetl(fid) to read current line
% it'll return -1 at the end of line

% CurrLine{Idx of row}{Idx of value you want}

NumLines = k-1; % Shows number of valid lines
Block = zeros(NumLines, 9); % Each row is a block
a = 1;

for k=1:NumLines
    if(strcmp(CurrLine{k}(1), 'boundary'))
        x0 = str2double(CurrLine{k}(2));
        y0 = str2double(CurrLine{k}(3));
        z0 = str2double(CurrLine{k}(4));
        x1 = str2double(CurrLine{k}(5));
        y1 = str2double(CurrLine{k}(6));
        z1 = str2double(CurrLine{k}(7));
    else
        if(strcmp(CurrLine{k}(1), 'block'))
            Block(a, 1) = str2double(CurrLine{k}(2))-margin;
            Block(a, 2) = str2double(CurrLine{k}(3))-margin;
            Block(a, 3) = str2double(CurrLine{k}(4))-margin;
            Block(a, 4) = str2double(CurrLine{k}(5))+margin;
            Block(a, 5) = str2double(CurrLine{k}(6))+margin;
            Block(a, 6) = str2double(CurrLine{k}(7))+margin;
            Block(a, 7) = 255;%str2double(CurrLine{k}(8)); % Color R Channel
            Block(a, 8) = 0;%str2double(CurrLine{k}(9)); % Color G Channel
            Block(a, 9) = 0;%str2double(CurrLine{k}{10}); % Color B Channel
            BlockOrg(a, 1) = str2double(CurrLine{k}(2));
            BlockOrg(a, 2) = str2double(CurrLine{k}(3));
            BlockOrg(a, 3) = str2double(CurrLine{k}(4));
            BlockOrg(a, 4) = str2double(CurrLine{k}(5));
            BlockOrg(a, 5) = str2double(CurrLine{k}(6));
            BlockOrg(a, 6) = str2double(CurrLine{k}(7));
            BlockOrg(a, 7) = 255;%str2double(CurrLine{k}(8)); % Color R Channel
            BlockOrg(a, 8) = 0;%str2double(CurrLine{k}(9)); % Color G Channel
            BlockOrg(a, 9) = 0;%str2double(CurrLine{k}{10}); % Color B Channel
            a = a+1;
        end
    end
end
NumBlocks = a-1; % Shows number of valid blocks


% hold on;
% grid on;
% % axis([0, ceil((x1-x0)/xy_res), 0, ceil((y1-y0)/xy_res), 0, ceil((z1-z0)/z_res)]);
% xlabel('X axis');
% ylabel('Y axis');
% zlabel('Z axis');

AllObsPts = [];
%% Discretize the blocks
BlockDiscrete = Block(1:NumBlocks, 1:9);
Lows =  repmat([x0, y0, z0], 1, 2)';

for i=1:6
    BlockDiscrete(:, i) = ceil((BlockDiscrete(:, i)-Lows(i))/DiscretizationFactor(i));%(= q * round(u/q)
end

for k = 1:NumBlocks
    a0 = BlockDiscrete(k, 1);
    a1 = BlockDiscrete(k, 4);
    b0 = BlockDiscrete(k, 2);
    b1 = BlockDiscrete(k, 5);
    c0 = BlockDiscrete(k, 3);
    c1 = BlockDiscrete(k, 6);
    ObsX = (a0:DiscretizationFactor(1):a1);
    ObsY = (b0:DiscretizationFactor(2):b1);
    ObsZ = (c0:DiscretizationFactor(3):c1);
    AllObsPts = [AllObsPts; FindAllLocs(ObsX, ObsY, ObsZ)];
end
hold on;
for k = 1:NumBlocks
a0 = BlockOrg(k, 1);
a1 = BlockOrg(k, 4);
b0 = BlockOrg(k, 2);
b1 = BlockOrg(k, 5);
c0 = BlockOrg(k, 3);
c1 = BlockOrg(k, 6);
RGB = Block(k, 7:9)./255;
X = [a0, a1, a1, a0, a1, a1, a0, a0];
Y = [b0, b0, b1, b1, b0, b1, b1, b0];
Z = [c0, c0, c0, c0, c1, c1, c1, c1];
FaceIds = [1:4];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), RGB);
FaceIds = [5:8];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), RGB);
FaceIds = [1, 2, 5, 8];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), RGB);
FaceIds = [2, 3, 6, 5];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), RGB);
FaceIds = [1, 4, 7, 8];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), RGB);
FaceIds = [1, 2, 5, 8];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), RGB);
FaceIds = [4, 3, 6, 7];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), RGB);
end


for k = 1:NumBlocks
a0 = Block(k, 1);
a1 = Block(k, 4);
b0 = Block(k, 2);
b1 = Block(k, 5);
c0 = Block(k, 3);
c1 = Block(k, 6);
X = [a0, a1, a1, a0, a1, a1, a0, a0];
Y = [b0, b0, b1, b1, b0, b1, b1, b0];
Z = [c0, c0, c0, c0, c1, c1, c1, c1];
FaceIds = [1:4];
Alpha = 0.2;
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), [0 1 0], 'FaceAlpha', Alpha);
FaceIds = [5:8];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), [0 1 0], 'FaceAlpha', Alpha);
FaceIds = [1, 2, 5, 8];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), [0 1 0], 'FaceAlpha', Alpha);
FaceIds = [2, 3, 6, 5];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), [0 1 0], 'FaceAlpha', Alpha);
FaceIds = [1, 4, 7, 8];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), [0 1 0], 'FaceAlpha', Alpha);
FaceIds = [1, 2, 5, 8];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), [0 1 0], 'FaceAlpha', Alpha);
FaceIds = [4, 3, 6, 7];
fill3(X(FaceIds), Y(FaceIds), Z(FaceIds), [0 1 0], 'FaceAlpha', Alpha);
end
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
% alpha(0.2);

NumBlocks = a-1; % Shows number of valid blocks
map = Block(1:NumBlocks, :);
map(end+1, :) = [x0, y0, z0, x1, y1, z1, 0, 0, 0];
map(end+1, :) = [xy_res, xy_res, z_res, 0, 0, 0, 0, 0, 0];
end
