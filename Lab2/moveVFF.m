clearvars -except tbot

% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end
addpath include/
map = read_map('maps/csquare_grid5.png');

n_points = input('How many points?');
figure(1);
title(['How many points?'])
figure(1);
[map_y,map_x] = find(map);
scatter(map_y./20,map_x./20,120,"black","filled")
axis([0, 4, 0, 4])                % the limits for the current axes [xmin xmax ymin ymax]
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
Ks = 0.6;

dk = 0.1;
tbot.setPose(0.5,0.5,-0);
[x,y,theta] = tbot.readPose();

x_= [];
y_=[];

i=1;
erroAnterior=0;

last_update = tic;

dist = 1;

for j = 1:n_points
    dist = 1;
    while (dist>0.1)
    
        plotPose(x,y,theta,x_,y_,map, goalPose, n_points);
    
        active_cells = getActiveArea([x,y],map,20);
        [world_x, world_y] = grid2world(active_cells(:,1),active_cells(:,2),size(map,1));
        active_cells_world = [world_x, world_y];
    
        [frx,fry,fox,foy]=VFF(tbot,world_x, world_y,goalPose(j,:));
    
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
    
    end
end

tbot.stop();