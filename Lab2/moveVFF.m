clearvars -except tbot

% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end
addpath include/
map = read_map('maps/fmap_grid5.png');

Kv = 0.6;
Ki = 0.2;
Ks = 0.5;

dk = 0.1;
tbot.resetPose();
tbot.setPose(0.5,0.5,-0);
[x,y,theta] = tbot.readPose();
goalPose = [3.5 3.5 pi/2];
dist = sqrt((goalPose(1)-x)^2+(goalPose(2)-y)^2);

x_= [];
y_=[];

i=1;
erroAnterior=0;

last_update = tic;

while (dist>0.1)

    plotPose(x,y,theta,x_,y_,map);

    active_cells = getActiveArea([x,y],map,30);
    [world_x, world_y] = grid2world(active_cells(:,1),active_cells(:,2),size(map,1));
    active_cells_world = [world_x, world_y];

    [frx,fry,fox,foy]=VFF(tbot,map,world_x, world_y,goalPose);

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
    %x_ = [x_ x];
    %y_ = [y_ y];
    dist = sqrt((goalPose(1)-x)^2+(goalPose(2)-y)^2);

end

tbot.stop();