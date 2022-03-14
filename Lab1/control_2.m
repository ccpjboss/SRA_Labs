% init TurtleBot connection (tbot object), if required
clearvars -except tbot
close all
global handle_v handle_w handle_d

if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end
tbot.resetPose();

hold on
figure(1);
axis([-2.5, 2.5, -2.5, 2.5])                % the limits for the current axes [xmin xmax ymin ymax]
grid on;                            % enable grid 
xlabel('x')                         % axis labels 
ylabel('y')
title(['Escolha 2 pontos para desenhar a linha'])
[px,py]=ginput(2);
title(['Movimento segundo uma linha'])
hold off
% 
reta = [py(1)-py(2), px(2)-px(1), px(1)*py(2)-px(2)*py(1)]; %Vetor reta (a,b,c) c/ ax+by+c=0
x_ = [];
y_ = [];

kd = 0.6;
kh = 0.5;

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
iter = 0;

while (1)
    [x,y,theta] = tbot.readPose();
    x_ = [x_ x];
    y_ = [y_ y];
    
    plot_pose(x, y, theta,reta,x_,y_,kd,kh);

    d = dot(reta,[x,y,1])/sqrt(reta(1)^2+reta(2)^2);
    phi = atan2(-reta(1),reta(2));

    alpha_d = -kd*d;
    alpha_h = kh*atan2(sin(phi-theta),cos(phi-theta));
    w = alpha_d+alpha_h;
    handle_v.XData=[handle_v.XData (iter-1)/10];
    handle_v.YData=[handle_v.YData 0.08];
    handle_w.XData=[handle_w.XData (iter-1)/10];
    handle_w.YData=[handle_w.YData w];
    handle_d.XData=[handle_d.XData (iter-1)/10];
    handle_d.YData=[handle_d.YData abs(d)];
    tbot.setVelocity(0.08,w);
    iter = iter +1;
end



function plot_pose(x, y, theta, reta, x_, y_,kd,kh)
    figure(1); clf; hold on;            % clear figure, hold plots
    title(['Kd:',num2str(kd),' Kh:',num2str(kh)])
    plot(x, y,'--or', 'MarkerSize', 10)  % display (x,y) location of the robot
    x_reta = -2.5:0.2:2.5;
    y_reta = (-reta(1).*x_reta-reta(3))./reta(2); 
    plot(x_reta,y_reta);
    quiver(x,y,cos(theta),sin(theta), 0.1, 'Color','r','LineWidth',1, 'ShowArrowHead',1)
    %line([0 x], [0 y], 'LineStyle', '--')
    plot(x_(1:5:end), y_(1:5:end),'--')
    quiver(0,0,1,0,'r')                 % draw arrow for x-axis 
    quiver(0,0,0,1,'g')                 % draw arrow for y-axis 
    axis([-2.5, 2.5, -2.5, 2.5])                % the limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')
    pause(0.1)
end