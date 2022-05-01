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

answer = inputdlg('How many points?');
n_points = str2double(answer{1});
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
% l0(1,:) = 0.2;
% l0(end,:) = 0.2;
% l0(:,1) = 0.2;
% l0(:,end) = 0.2;

figure(2); 
map_handle = imshow(zeros(80));
v = 0;
for j=1:n_points
    dist = sqrt((goalPose(j,1)-x)^2+(goalPose(j,2)-y)^2);
    while (dist>0.15)
        plotPose(x,y,theta,x_,y_,map,goalPose,n_points);

        [scan, lddata, ldtimestamp] = tbot.readLidar();
        [x,y,theta,timestamp] = tbot.readPose();
        % compute the time diference between the pose reading and laser scan (in seconds) 
        timeDiff =  timestamp - ldtimestamp;
    
        % Get in range lidar data indexs (valid data - obstacles).
        [vidx] = tbot.getInRangeLidarDataIdx(lddata);
        % load valid X/Y cartesian lidar readings          
        xydata = lddata.Cartesian(vidx,:);
    
        % Get out of range lidar data indexs (too near or too far readings).
        [nidx] = tbot.getOutRangeLidarDataIdx(lddata);
        % load obstacle free orientations (angles)  
        freeAng = lddata.Angles(nidx);

        [robotX, robotY] = world2grid(x,y,80);

        transformMatrix = [cos(theta) -sin(theta) x-0.0305-(v*timeDiff*cos(theta));
    			           sin(theta) cos(theta) y-(v*timeDiff*sin(theta));
			               0 0 1];

        xyWorld = transformMatrix*[xydata';ones(1,size(xydata,1))];
        xyWorld = xyWorld(1:2,:);
        [xCell, yCell] = world2grid(xyWorld(1,:),xyWorld(2,:),80);
        map = updateMap(map,xCell,yCell,[robotX, robotY,theta],l0,freeAng, transformMatrix);
        active_cells = getActiveArea([x,y],map,xCell', yCell',robotX, robotY,window_size);
        [world_x, world_y] = grid2world(active_cells(:,1),active_cells(:,2),size(map,1));
    
        h_smooth = getPolarHistogram(world_y, y, world_x, x, theta, alpha,window_size);    
        [go_theta_deg] = getSteeringDirection(h_smooth, goalPose(j,:), x, y, alpha, theta);
        
        go_theta_rad = deg2rad(go_theta_deg(1));
        go_theta_rad = go_theta_rad+2*pi*(go_theta_rad<0);
    
        nPose(1) = x + 0.3*cos(go_theta_rad);
        nPose(2) = y + 0.3*sin(go_theta_rad);

        figure(2);
        map_handle.CData = flip(map,1); %imshow(flip(map,1),[])
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
    
        x_ = [x_ x];
        y_ = [y_ y];
        dist = sqrt((goalPose(j,1)-x)^2+(goalPose(j,2)-y)^2);
    end
end
figure(2);
subplot(1,2,1)
imshow(flip(map,1));
subplot(1,2,2)
colormap('gray')
surface(1:80,1:80,map);
axis([1 80 1 80]);
title('Final Probabilistic Map')
tbot.stop();
