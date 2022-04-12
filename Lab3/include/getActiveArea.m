function [active_cells] = getActiveArea(robotPose,map,xCell, yCell,robot_x_cell, robot_y_cell, N)

%[robot_x_cell, robot_y_cell] = world2grid(robotPose(1), robotPose(2),size(map,1));

low_limit_x = max(1,floor(robot_x_cell-N/2));
high_limit_x = min(size(map,1),floor(robot_x_cell+N/2));
low_limit_y = max(1,floor(robot_y_cell-N/2));
high_limit_y = min(size(map,1),floor(robot_y_cell+N/2));


cond_x = (xCell>low_limit_x & xCell<high_limit_x);
cond_y = (yCell>low_limit_y & yCell<high_limit_y);
cell_x = xCell(cond_x & cond_y);
cell_y = yCell(cond_x & cond_y);
active_cells = [cell_x, cell_y];


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

