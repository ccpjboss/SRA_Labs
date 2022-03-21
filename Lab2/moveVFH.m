close all
clearvars -except tbot
addpath include/

%init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end

tbot.setPose(0.5,0.5,pi/2);
goal_pose = [2.5,2.5];
dk = 0.1;

map = read_map("./maps/fmap_grid5.png");
[x,y,theta] = tbot.readPose();
dist = sqrt((goal_pose(1)-x)^2+(goal_pose(2)-y)^2);
alpha = deg2rad(10);
ek_anterior = 0;
delta_t = tic;

kv = 0.5;
ki = 0.2;
ks = 0.7;

x_ = [];
y_ = [];
h_smooth = [];

while (dist>0.05)
    plotPose(x,y,theta,x_,y_,map);
    active_cells = getActiveArea([x,y],map,30);
    [world_x, world_y] = grid2world(active_cells(:,1),active_cells(:,2),size(map,1));
    active_cells_world = [world_x, world_y];

    h_smooth = getPolarHistogram(world_y, y, world_x, x, theta, alpha);
    drawHistogram(h_smooth);

    [go_theta_deg] = getSteeringDirection(h_smooth, goal_pose, x, y, alpha);
    go_theta_rad = deg2rad(go_theta_deg);
    
    nPose(1) = x + 0.25*cos(go_theta_rad);
    nPose(2) = y + 0.25*sin(go_theta_rad);

    figure(1);
    hold on;
    plot(nPose(1),nPose(2),'ro');

    ek = sqrt((nPose(1)-x)^2+(nPose(2)-y)^2) - dk;
    integral = ((ek+ek_anterior)/2)*toc(delta_t); 
    delta_t = tic;
    ek_anterior = ek;
    v = kv*ek+ki*integral;
    nPose(3) = atan2((nPose(2)-y),(nPose(1)-x));
    w = ks*atan2(sin(nPose(3)-theta),cos(nPose(3)-theta));
    
    tbot.setVelocity(v,w);

    [x,y,theta] = tbot.readPose();
    x_ = [x_ x];
    y_ = [y_ y];
    dist = sqrt((goal_pose(1)-x)^2+(goal_pose(2)-y)^2);
end

tbot.stop();