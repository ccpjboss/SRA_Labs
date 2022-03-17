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
window_cm = N/2*5;
x1=robotPose(1)-window_cm/100;
y1=robotPose(2)-window_cm/100;
x2=robotPose(1)+window_cm/100;
y2=robotPose(2)+window_cm/100;

sx = [x1, x2, x2, x1, x1];
sy = [y1, y1, y2, y2, y1];

figure(1)
hold on 
plot(sx, sy,'b--',"LineWidth",2)

end

