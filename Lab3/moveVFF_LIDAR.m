clearvars -except tbot
close all

% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end
addpath include/
map = zeros(80);
map(:) = 0.5;

answer = inputdlg('How many points?');
n_points = str2double(answer{1});
figure(1);
axis([0, 4, 0, 4])
grid on;             
[xi,yi]=ginput(n_points);

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

Kv = 0.2;
Ki = 0.3;
Ks = 0.7;

dk = 0.1;
tbot.setPose(0.5,0.5,-0);
[x,y,theta] = tbot.readPose();

x_= [];
y_=[];

nPose = [0 0 0];

i=1;
erroAnterior=0;

last_update = tic;

dist = 1;

active_cells = [];
l0 = zeros(80);
% l0(1,:) = 0.2;
% l0(end,:) = 0.2;
% l0(:,1) = 0.2;
% l0(:,end) = 0.2;

figure(2); 
map_handle = imshow(zeros(80));
v = 0;
for j = 1:n_points
    dist = 1;
    while (dist>0.2) 
        % load lidar data from robot 
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
        map = updateMap(map,xCell,yCell,[robotX, robotY,theta],l0,freeAng,transformMatrix);

        plotPose(x,y,theta,x_,y_,map, goalPose, n_points);

        [frx,fry,fox,foy]=VFF(tbot,xyWorld(1:2,:)',goalPose(j,:));
    
        nPose(1) = x + frx;
        nPose(2) = y + fry;
    
        error = norm(nPose(1:2) - [x, y]) - dk;
    
        vRobot = Kv*error+Ki*((error-erroAnterior)/2)*toc(last_update);
        if (vRobot > 3.5)
            vRobot = 3.5;
        end
        last_update = tic;
    
        erroAnterior = error; %Erro anterior para o caluculo da velocidade
    
        % Computes angular velocity
        nPose(3) = atan2(nPose(2)-y,nPose(1)-x);
        wRobot = Ks*atan2(sin(nPose(3)-theta),cos(nPose(3)-theta));
        tbot.setVelocity(vRobot, wRobot);
    
        x_ = [x_ x];
        y_ = [y_ y];
        dist = sqrt((goalPose(j,1)-x)^2+(goalPose(j,2)-y)^2);

        figure(2);
        map_handle.CData = flip(map,1); %imshow(flip(map,1),[])
        title('map')

        figure(1)
        hold on
        hold off
        
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

colorbar

tbot.stop();