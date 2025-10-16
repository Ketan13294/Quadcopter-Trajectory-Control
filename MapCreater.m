map = load("exampleMaps.mat").simpleMap;
map3 = [];
res = 0.25;
for k = 1:25
    for i = 1:26
        for j = 1:27
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
insertPointCloud(map3D,pose,map3,maxRange)
save('myMap3D.mat', 'map3D');
show(map3D)
grid on;
ss = stateSpaceSE3;
ss.StateBounds = [0,10;0,10;0,6;-1,1;-1,1;-1,1;-1,1];
sv = validatorOccupancyMap3D(ss,Map=map3D);
sv.ValidationDistance = 0.1;
checkOccupancy(map3D,[1,2,1])
checkOccupancy(map3D,goal(1:3))
sv.isStateValid(start)
sv.isStateValid(goal)
%%
% Clear environment
clear; clc;

% --- 1. Create a 3D occupancy map ---
res = 0.1;                % 0.1 m per voxel
map3D = occupancyMap3D(5);

% Define wall dimensions
wallHeight = 2;           % meters
xMax = 10;
yMax = 10;


[x, y, z] = meshgrid(0:res:xMax, 0:res:yMax, 0:res:wallHeight);
setOccupancy(map3D, [x(:), y(:), z(:)], 0);  % mark all as free
% --- 2. Build walls using setOccupancy ---
% Each wall will be represented by occupied voxels along a plane

% Generate Z coordinates (0 to wallHeight)
zVals = 0:res:wallHeight;

% Wall at X = 0 (Y from 0 to 10)
[y,z] = meshgrid(0:res:yMax, zVals);
x = zeros(size(y));
setOccupancy(map3D, [x(:), y(:), z(:)], 1);

% Wall at X = 10
x = xMax*ones(size(y));
setOccupancy(map3D, [x(:), y(:), z(:)], 1);

% Wall at Y = 0
[x,z] = meshgrid(0:res:xMax, zVals);
y = zeros(size(x));
setOccupancy(map3D, [x(:), y(:), z(:)], 1);

% Wall at Y = 10
y = yMax*ones(size(x));
setOccupancy(map3D, [x(:), y(:), z(:)], 1);

% --- 3. Visualize the map ---
figure;
show(map3D);
title('3D Occupancy Map with 2m Walls at Boundaries');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

% --- 4. Check occupancy of points ---
p1 = [5, 5, 1];
p2 = [1, 1, 1];

occ1 = checkOccupancy(map3D, p1);
occ2 = checkOccupancy(map3D, p2);

fprintf('(5,5,2) occupancy = %.1f (0=free, 1=occupied, -1=out of bounds)\n', occ1);
fprintf('(1,1,2) occupancy = %.1f (0=free, 1=occupied, -1=out of bounds)\n', occ2);

% --- 5. Visualize test points ---
hold on;
plot3(p1(1), p1(2), p1(3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(p2(1), p2(2), p2(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
legend('Walls','Point (5,5,2)','Point (1,1,2)');
