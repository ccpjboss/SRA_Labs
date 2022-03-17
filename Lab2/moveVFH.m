close all
clearvars -except tbot
addpath include/

% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end

tbot.setPose(1.5,2.5,pi/2);
goal_pose = [1.5,3.5];

map = read_map("./maps/fmap_grid5.png");
[x,y,theta] = tbot.readPose();
dist = sqrt((goal_pose(1)-x)^2+(goal_pose(2)-y)^2);
alpha = deg2rad(10);

% while (dist<0.05)
    [x,y,theta] = tbot.readPose();
    plotPose(x,y,theta,map);
    active_cells = getActiveArea([x,y],map,30);
    [world_x, world_y] = grid2world(active_cells(:,1),active_cells(:,2),size(map,1));
    active_cells_world = [world_x, world_y];

    h_smooth = getPolarHistogram(world_y, y, world_x, x, alpha);
    
    figure(2); bar(h_smooth);
    [go_theta_deg] = getSteeringDirection(h_smooth, goal_pose, y, x, alpha);

% end