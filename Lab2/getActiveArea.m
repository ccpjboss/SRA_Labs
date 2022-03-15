function [activeArea] = getActiveArea(robotPose,map,N)
%GETACTIVEAREA Summary of this function goes here
%   Detailed explanation goes here

robot_x_cell = round(size(map,1)*robotPose(1)/4);
robot_y_cell = round(size(map,1)*robotPose(2)/4);

low_limit_x = max(1,floor(robot_x_cell-N/2));
high_limit_x = min(size(map,1),floor(robot_x_cell+N/2));
low_limit_y = max(1,floor(robot_y_cell-N/2));
high_limit_y = min(size(map,1),floor(robot_y_cell+N/2));

activeArea = map(low_limit_y:high_limit_y,low_limit_x:high_limit_x);
end

