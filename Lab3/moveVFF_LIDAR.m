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

n_points = input('How many points?');
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

Kv = 0.5;
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
l0(1,:) = 0.2;
l0(end,:) = 0.2;
l0(:,1) = 0.2;
l0(:,end) = 0.2;

for j = 1:n_points
    dist = 1;
    while (dist>0.1) 
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

        plotPose(x,y,theta,x_,y_,map, goalPose, n_points, nPose);

        [frx,fry,fox,foy]=VFF(tbot,xyWorld(1:2,:)',goalPose(j,:));
    
        nPose(1) = x + frx;
        nPose(2) = y + fry;
    
        error = norm(nPose(1:2) - [x, y]) - dk;
    
        vRobot = Kv*error+Ki*((error-erroAnterior)/2)*toc(last_update);
        last_update = tic;
    
        erroAnterior = error; %Erro anterior para o caluculo da velocidade
    
        % Computes angular velocity
        nPose(3) = atan2(nPose(2)-y,nPose(1)-x);
        wRobot = Ks*atan2(sin(nPose(3)-theta),cos(nPose(3)-theta));
        tbot.setVelocity(vRobot, wRobot);
    
        [x,y,theta] = tbot.readPose();
        x_ = [x_ x];
        y_ = [y_ y];
        dist = sqrt((goalPose(j,1)-x)^2+(goalPose(j,2)-y)^2);

        figure(2);
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
%         imshow(rot90(imcomplement(map)),[])
        imshow(rot90(map),[])

        title('map')
        
    end
end

tbot.stop();