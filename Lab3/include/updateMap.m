function [newMap] = updateMap(map,cellX,cellY,robotPose,l0)

% Create log odds map for update
log_map = log(map./(1-map));

for i= 1:size(cellX,2)
    % Update occupied cells
    log_map(cellY(i),cellX(i)) = log_map(cellY(i),cellX(i)) + 0.65;
    % Get free cells with bresenham algorithm
    [lineX, lineY] = bresenham(robotPose(1),robotPose(2),cellX(i),cellY(i));
    % Delete the last cell that corresponds to the obstacle cell
    freeCells = [lineX(1:end-1) lineY(1:end-1)];

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

