function [newMap] = updateMap(map,cellX,cellY,robotPose,l0,freeAng,transformMatrix)
map_size = size(map,1);
% [robot_world_x, robot_world_y] = grid2world(robotPose(1),robotPose(2),map_size);

% Get the free Cells
freeXY = [1.5*cos(freeAng) 1.5*sin(freeAng)];

xyWorld = transformMatrix*[freeXY';ones(1,size(freeXY,1))];
xyWorld = xyWorld(1:2,:);

% lidar_plot = zeros(2,2*size(xyWorld,2));
% lidar_start_points_angles = linspace(0,2*pi,size(xyWorld,2));
% %lidar_plot(:,1:2:end) = repmat([robot_world_x;robot_world_y],[1,size(xyWorld,2)]);
% lidar_plot(:,1:2:end) = [robot_world_x+0.12*cos(lidar_start_points_angles);robot_world_y+0.12*sin(lidar_start_points_angles)];
% lidar_plot(:,2:2:end) = xyWorld;
% 
% figure(1);
% line(lidar_plot(1,:),lidar_plot(2,:),'LineWidth',0.05);

[freeAX, freeAY] = world2grid(xyWorld(1,:),xyWorld(2,:),map_size);
freeAX = min(max(freeAX,1),map_size);
freeAY  = min(max(freeAY,1),map_size);

% Create log odds map for update
log_map = log(map./(1-map));

for i = 1:size(freeAX,2)
    [lineAX, lineAY] = bresenham(robotPose(1),robotPose(2),freeAX(i),freeAY(i));
    lineAX = min(max(lineAX,1),map_size);
    lineAY  = min(max(lineAY,1),map_size);
    for j = 1:size(lineAX,1)
        % Update the free cells
        log_map(lineAY(j),lineAX(j)) = log_map(lineAY(j),lineAX(j)) - 0.65;
    end
end

cellX = min(max(cellX,1),map_size);
cellY  = min(max(cellY,1),map_size);

for i= 1:size(cellX,2)
    % Update occupied cells
    log_map(cellY(i),cellX(i)) = log_map(cellY(i),cellX(i)) + 0.65;
    % Get free cells with bresenham algorithm

    [lineX, lineY] = bresenham(robotPose(1),robotPose(2),cellX(i),cellY(i));

    % Delete the last cell that corresponds to the obstacle cell
    freeCells = [lineX(1:end-1) lineY(1:end-1)];
    freeCells(:,1) = min(max(freeCells(:,1),1),map_size);
    freeCells(:,2)  = min(max(freeCells(:,2),1),map_size);
    for j = 1:size(freeCells,1)
        % Update the free cells
        log_map(freeCells(j,2),freeCells(j,1)) = log_map(freeCells(j,2),freeCells(j,1)) - 0.65;
    end
end

% Sum prior to the log map so that the borders start updating
log_map = log_map + l0;

% Convert the map to probailities values
newMap = 1-1./(1+exp(log_map));
end

