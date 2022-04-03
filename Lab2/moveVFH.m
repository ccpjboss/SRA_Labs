close all
clearvars -except tbot
addpath include/

%init TurtleBot connection (tbot object), if required
if ( ~exist("tbot",'var') ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end

map = read_map("./maps/umap_grid5.png");

n_points = input('How many points?');
figure(1);
title(('How many points?'))
figure(1);
[map_y,map_x] = find(map);
scatter(map_y./20,map_x./20,120,"black","filled")
axis([0, 4, 0, 4])                % the limits for the current axes [xmin xmax ymin ymax]
grid on;             
[xi,yi]=ginput(n_points);
window_size = 30;

hold off
for j=1:n_points
    goalPose(j,1) = xi(j);
    goalPose(j,2) = yi(j);
    hold on
    if (j == n_points)
        plot(goalPose(j,1),goalPose(j,2),'gx', 'MarkerSize', 5); %display locations of points
    else
        plot(goalPose(j,1),goalPose(j,2),'bx', 'MarkerSize', 5); %display locations of points
    end
end


tbot.setPose(2,0.7,pi/2);
% goal_pose = [3.5,3.5];
dk = 0.1;

[x,y,theta] = tbot.readPose();
alpha = deg2rad(10);
ek_anterior = 0;
delta_t = tic;

kv = 0.5;
ki = 0.2;
ks = 0.7;

x_ = [];
y_ = [];
h_smooth = [];

for j=1:n_points
    dist = sqrt((goalPose(j,1)-x)^2+(goalPose(j,2)-y)^2);
    while (dist>0.15)
        plotPose(x,y,theta,x_,y_,map,goalPose,n_points);
        active_cells = getActiveArea([x,y],map,window_size);
        [world_x, world_y] = grid2world(active_cells(:,1),active_cells(:,2),size(map,1));
        active_cells_world = [world_x, world_y];
    
        h_smooth = getPolarHistogram(world_y, y, world_x, x, theta, alpha,window_size);
        drawHistogram(h_smooth);
    
        [go_theta_deg] = getSteeringDirection(h_smooth, goalPose(j,:), x, y, alpha, theta);
        
        go_theta_rad = deg2rad(go_theta_deg(1));
        go_theta_rad = go_theta_rad+2*pi*(go_theta_rad<0);
    
        nPose(1) = x + 0.3*cos(go_theta_rad);
        nPose(2) = y + 0.3*sin(go_theta_rad);
    
        figure(1);
        hold on;
        plot(nPose(1),nPose(2),'ro');
        hold off
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
        dist = sqrt((goalPose(j,1)-x)^2+(goalPose(j,2)-y)^2);
    end
end

tbot.stop();
