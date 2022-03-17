clearvars -except tbot

% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end
addpath include/
map = read_map('maps/csquare_grid5.png');

Kv = 0.3;
Ki = 0.5;
Ks = 0.5;

dk = 2;

goalPose = [1 1 pi/2];
x_= [];
y_=[];

i=1;
erroAnterior=0;

last_update = tic;

dist = 1;

while dist > 0.05

    [x, y, theta] = tbot.readPose();
    x = x * 100;
    y = y * 100;
    dist = norm(goalPose(1:2)-[x, y]);

    [frx,fry,fox,foy]=VFF(tbot,map,goalPose);

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
    x_ = [x_ x];
    y_ = [y_ y];

end

tbot.resetPose();

function plot_pose(x, y, theta, goal_pose, x_, y_)
    figure(1); clf; hold on;            % clear figure, hold plots
    plot(x, y,'--or', 'MarkerSize', 10)  % display (x,y) location of the robot
    plot(goal_pose(1), goal_pose(2),'bx', 'MarkerSize', 5)
    quiver(x,y,cos(theta),sin(theta), 0.1, 'Color','r','LineWidth',1, 'ShowArrowHead',1)
    %line([0 x], [0 y], 'LineStyle', '--')
    plot(x_(1:5:end), y_(1:5:end),'--')
    quiver(0,0,1,0,'r')                 % draw arrow for x-axis 
    quiver(0,0,0,1,'g')                 % draw arrow for y-axis 
    axis([-2, 2, -2, 2])                % the limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')
    pause(0.1)
end