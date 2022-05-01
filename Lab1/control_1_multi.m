clearvars -except tbot
close all

global handle_v handle_w handle_d

n_points = input('How many points?');
figure(1);
title(['How many points?'])
grid
axis([-2, 2, -2, 2])                
[xi,yi]=ginput(n_points);

hold off
for j=1:n_points
    goal_pose(j,1) = xi(j);
    goal_pose(j,2) = yi(j);
    hold on
    plot(goal_pose(j,1),goal_pose(j,2),'bx', 'MarkerSize', 5);
end

figure(2)
subplot(3,1,1)
handle_v = plot(0,0);
title(['Velocidade Linear'])
grid on
subplot(3,1,2)
handle_w = plot(0,0);
title(['Velocidade Angular'])
grid on
subplot(3,1,3)
handle_d = plot(0,0);
title(['Distancia'])
grid on

% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end 
tbot.resetPose();
kv = 0.15;
ks = 0.5;

dist = 1;
x_= [];
y_= [];
iter = 0;
 
for j = 1:n_points
    dist = 1;
    while (dist > 0.1)
        % read TurtleBot pose
        [x,y,theta] = tbot.readPose();
    
        x_ = [x_ x];
        y_ = [y_ y];
        plot_pose(x, y,theta, goal_pose, x_, y_,ks,kv);
        dist = sqrt((goal_pose(j,1)-x)^2+(goal_pose(j,2)-y)^2);
        v = kv*dist;
        phi = atan2((goal_pose(j,2)-y),(goal_pose(j,1)-x));   
        w = ks*atan2(sin(phi-theta),cos(phi-theta));
        tbot.setVelocity(v,w);
        
        handle_v.XData=[handle_v.XData (iter-1)/10];
        handle_v.YData=[handle_v.YData v];
        handle_w.XData=[handle_w.XData (iter-1)/10];
        handle_w.YData=[handle_w.YData w];
        handle_d.XData=[handle_d.XData (iter-1)/10];
        handle_d.YData=[handle_d.YData abs(dist)];
        iter=iter+1;
    end
end
%save figure
tbot.resetPose();
pause(1)
tbot.resetPose();
        
        


function plot_pose(x, y, theta, goal_pose, x_, y_,ks,kv)
    figure(1); clf; hold on;            % clear figure, hold plots
    title(['Ks=',num2str(ks),' Kv=', num2str(kv)])
    plot(x, y,'--or', 'MarkerSize', 10)  % display (x,y) location of the robot
    plot(goal_pose(:,1), goal_pose(:,2),'bx', 'MarkerSize', 5)
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

