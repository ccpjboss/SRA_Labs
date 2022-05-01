function [newMap] = updateMap(map,cellX,cellY,robotPose,l0,freeAng,transformMatrix)

% Get the free Cells
freeXY = [1.5*cos(freeAng) 1.5*sin(freeAng)];

xyWorld = transformMatrix*[freeXY';ones(1,size(freeXY,1))];
xyWorld = xyWorld(1:2,:);
[freeAX, freeAY] = world2grid(xyWorld(1,:),xyWorld(2,:),80);

freeAX = min(max(freeAX,1),80);
freeAY  = min(max(freeAY,1),80);

% Create log odds map for update
log_map = log(map./(1-map));

for i = 1:size(freeAX,2)
    [lineAX, lineAY] = bresenham(robotPose(1),robotPose(2),freeAX(i),freeAY(i));
    lineAX = min(max(lineAX,1),80);
    lineAY  = min(max(lineAY,1),80);
    for j = 1:size(lineAX,1)
        % Update the free cells
        log_map(lineAY(j),lineAX(j)) = log_map(lineAY(j),lineAX(j)) - 0.65;
    end
end

cellX = min(max(cellX,1),80);
cellY  = min(max(cellY,1),80);

for i= 1:size(cellX,2)
    % Update occupied cells
    log_map(cellY(i),cellX(i)) = log_map(cellY(i),cellX(i)) + 0.65;
    % Get free cells with bresenham algorithm

    [lineX, lineY] = bresenham(robotPose(1),robotPose(2),cellX(i),cellY(i));

    % Delete the last cell that corresponds to the obstacle cell
    freeCells = [lineX(1:end-1) lineY(1:end-1)];
    freeCells(:,1) = min(max(freeCells(:,1),1),80);
    freeCells(:,2)  = min(max(freeCells(:,2),1),80);
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

