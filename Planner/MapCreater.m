map = load("exampleMaps.mat").simpleMap;

%%
map3 = [];
[xMax,yMax] = size(map);
res = 0.25;
for k = 1:10
    for i = 1:xMax
        for j = 1:yMax
            if ( k==1)
                map3 = [map3;res*(i-1),res*(j-1),res*(k-1)];
            elseif (map(i,j) == 1)
                map3 = [map3;res*(i-1),res*(j-1),res*(k-1)];
            end
        end
    end
end

start = [2,2,2,1,0,0,0];
goal = [5.5,5,3,1,0,0,0];
% map3 = [map3];

map3D = occupancyMap3D(4);
[x, y, z] = meshgrid(0:res:7, 0:res:7, 0:res:7);
setOccupancy(map3D, [x(:), y(:), z(:)], 0);
setOccupancy(map3D, map3, 1);
pose = [0 0 0 1 0 0 0];
maxRange = 60;
% insertPointCloud(map3D,pose,map3,maxRange)
save('myMap3D.mat', 'map3D');

%%
mapoccup = occupancyMap(map,4);
inflate(mapoccup,0.2);
map = mapoccup.occupancyMatrix("ternary")
map3 = [];
res = 0.25;
[xMax,yMax] = size(map);
for k = 1:10
    for i = 1:xMax
        for j = 1:yMax
            if ( k==1)
                map3 = [map3;res*(i-1),res*(j-1),res*(k-1)];
            elseif (map(i,j) == 1)
                map3 = [map3;res*(i-1),res*(j-1),res*(k-1)];
            end
        end
    end
end

start = [2,2,2,1,0,0,0];
goal = [5.5,5,3,1,0,0,0];
% map3 = [map3];

map3D = occupancyMap3D(4);
[x, y, z] = meshgrid(0:res:7, 0:res:7, 0:res:7);
setOccupancy(map3D, [x(:), y(:), z(:)], 0);
setOccupancy(map3D, map3, 1);
pose = [0 0 0 1 0 0 0];
maxRange = 60;
% insertPointCloud(map3D,pose,map3,maxRange)
save('myMap3D_inflate.mat', 'map3D');
