close all
clearvars -except tbot
addpath include/

%init TurtleBot connection (tbot object), if required
if ( ~exist("tbot",'var') ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end

map = zeros(80);
map(:) = 0.5;

n_points = input('How many points?');
figure(1);
title(('How many points?'))
figure(1);
axis([0, 4, 0, 4]) % the limits for the current axes [xmin xmax ymin ymax]
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


tbot.setPose(0.5,0.5,pi/2);
% goal_pose = [3.5,3.5];
dk = 0.1;

[x,y,theta] = tbot.readPose();
alpha = deg2rad(10);
ek_anterior = 0;
delta_t = tic;

kv = 0.5;
ki = 0.2;
ks = 0.7;

nPose = [0 0 0];

x_ = [];
y_ = [];
h_smooth = [];

l0 = zeros(80);
l0(1,:) = 0.2;
l0(end,:) = 0.2;
l0(:,1) = 0.2;
l0(:,end) = 0.2;

for j=1:n_points
    dist = sqrt((goalPose(j,1)-x)^2+(goalPose(j,2)-y)^2);
    while (dist>0.15)
        plotPose(x,y,theta,x_,y_,map,goalPose,n_points, nPose);

        [scan, xydata, angles] = tbot.readLidar();
        [x,y,theta] = tbot.readPose();
        [robotX, robotY] = world2grid(x,y,80);

        transformMatrix = [cos(theta) -sin(theta) x-0.0305;
    			           sin(theta) cos(theta) y;
			               0 0 1];

        xyWorld = transformMatrix*[xydata';ones(1,size(xydata,1))];
        xyWorld = xyWorld(1:2,:);
        [xCell, yCell] = world2grid(xyWorld(1,:),xyWorld(2,:),80);
        map = updateMap(map,xCell,yCell,[robotX, robotY],l0);
        active_cells = getActiveArea([x,y],map,xCell', yCell',robotX, robotY,window_size);
        [world_x, world_y] = grid2world(active_cells(:,1),active_cells(:,2),size(map,1));
    
        h_smooth = getPolarHistogram(world_y, y, world_x, x, theta, alpha,window_size);
        drawHistogram(h_smooth);
    
        [go_theta_deg] = getSteeringDirection(h_smooth, goalPose(j,:), x, y, alpha, theta);
        
        go_theta_rad = deg2rad(go_theta_deg(1));
        go_theta_rad = go_theta_rad+2*pi*(go_theta_rad<0);
    
        nPose(1) = x + 0.3*cos(go_theta_rad);
        nPose(2) = y + 0.3*sin(go_theta_rad);

        figure(3);
        subplot(2,2,1);
        rosPlot(scan);
        subplot(2,2,2);
        plot(xyWorld(1,:)',xyWorld(2,:)');
        title('Laser Scan in World Frame')
        axis([0 4 0 4]);
        grid minor;
        subplot(2,2,3);
        plot(xCell,yCell);
        title('Discrete map')
        axis([0 80 0 80]);
        grid minor;
        subplot(2,2,4);
        imshow(rot90(imcomplement(map)),[])
        title('map')
    
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
