%% user input vars
m = 10; n=10; p=10; % world dimensions, where mxnxp ---> l x w x h 
radius = 1; %obstacle size
num_obstacles = 5; %number of obstacles

%% create obstacle map
% set world 
[px,py,pz] = meshgrid(1:n, 1:m, 1:p);
% set obstacles
i=0;
X = zeros(m,n,p);

x = [5 3.3 3.3 6.5 6.5];
y = [5 3.3 6.5 3.3 6.5];
z = [3 6.5 6.5 6.5 6.5];

% the center of sphere
while i<num_obstacles
%     xc=randi([radius,m-radius],1); %generate x coordinate, so that sphere doesn't touch edges
%     yc=randi([radius,n-radius],1);
%     zc =randi([radius,p-radius],1);
    xc = x(i+1);
    yc = y(i+1);
    zc = z(i+1);
    logicalSphere = (px-xc).^2 + (py-yc).^2 + (pz-zc).^2 <= radius*radius;
    if ~(ismember(1, X(logicalSphere))) 
        X(logicalSphere) = 1; % set as obstacle
        i=i+1;
    end 
end 

X = logical(X);
filename ='logical3DMap.mat';
save(filename, 'X')
%% visualize obstacle map
axis([0,n,0,m,0,p])
p=patch(isosurface(px,py,pz,X,0));
set(p, 'FaceColor', 'red', 'EdgeColor', 'none');
daspect([1 1 1])
view(3)
hold on 
scatter3(px(:),py(:),pz(:),1,X(:),'filled')
camlight; lighting phong
hold off

%% create occupancy map
omap = occupancyMap3D(5); %map resolution 10 cells/meter

% find coords where obstacles are 
[x,y,z]= ind2sub(size(X),find(X == 1));
%[x, y, z] = find(X==1);
xyzObstacles = [x(:) y(:) z(:)];
updateOccupancy(omap,xyzObstacles, 1)
figure(2)
show(omap)
