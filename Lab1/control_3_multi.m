clearvars -except tbot
close all
n_points = input('How many points?');
global handle_v handle_w handle_alpha handle_rho handle_beta

figure(1);
grid
axis([-2, 2, -2, 2])                
hold on
title(['Escolha a posição e orientação de ' num2str(n_points) ' poses'])
%Posição final arbitrária
[xi,yi]=ginput(2*n_points);
title(['Movimento para uma pose arbitrária'])
hold off
d=0;
for j=1:2:2*n_points
    d=d+1;
    Theta=atan2(yi(j+1)-yi(j),xi(j+1)-xi(j));
    goal_pose(d,1) = xi(j);
    goal_pose(d,2) = yi(j);
    goal_pose(d,3) = Theta;
    hold on;
    quiver(xi(j),yi(j),xi(j+1)-xi(j),yi(j+1)-yi(j),0);
end

figure(2)
subplot(2,1,1)
handle_v = plot(0,0);
title(['Velocidade Linear'])
grid on
subplot(2,1,2)
handle_w = plot(0,0);
title(['Velocidade Angular'])
grid on

figure(3)
subplot(3,1,1)
handle_alpha = plot(0,0);
title('Evolução de alpha')
grid on
subplot(3,1,2)
handle_rho = plot(0,0);
title('Evolução de rho')
grid on
subplot(3,1,3)
handle_beta = plot(0,0);
title('Evolução de beta')
grid on

last_update=tic;

k_rho = 0.2;
k_beta = -0.4;
k_alpha = 1;

x_ = [];
y_ = [];

% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end 
tbot.resetPose();
[x,y,theta]=tbot.readPose();
iter=1;
rho_=1;
beta_=1;
alpha_=1;
for j=1:1:d
    while (rho_>0.05 || abs(beta_)>0.15 || abs(alpha_)>0.15)
        [x,y,theta]=tbot.readPose();
        dx = goal_pose(j,1) - x;
        dy = goal_pose(j,2) - y;
        rho_ = sqrt(dx^2 + dy^2);
    
        alpha_ = -theta + atan2(dy,dx);
        alpha_ = atan2(sin(alpha_),cos(alpha_));
    
        beta_ = -theta - alpha_ + goal_pose(j,3);
        beta_ = atan2(sin(beta_),cos(beta_));
        
    
        [rho_,alpha_,beta_] = update_parameters(rho_,alpha_,beta_,k_rho,k_alpha,k_beta,last_update);
        last_update = tic;
        v = k_rho*rho_;
        w = k_alpha*alpha_+k_beta*beta_;
        plot_pose(x,y,theta,goal_pose(j,:),x_,y_,k_rho,k_alpha,k_beta);
        
        handle_v.XData=[handle_v.XData (iter-1)/10];
        handle_v.YData=[handle_v.YData v];
        handle_w.XData=[handle_w.XData (iter-1)/10];
        handle_w.YData=[handle_w.YData w];
        handle_alpha.XData=[handle_alpha.XData (iter-1)/10];
        handle_alpha.YData=[handle_alpha.YData alpha_];
        handle_rho.XData=[handle_rho.XData (iter-1)/10];
        handle_rho.YData=[handle_rho.YData rho_];
        handle_beta.XData=[handle_beta.XData (iter-1)/10];
        handle_beta.YData=[handle_beta.YData beta_];

        tbot.setVelocity(v,w);
         
        x_ = [x_ x];
        y_ = [y_ y];
        iter = iter + 1;
    end
    rho_=1;
    beta_=1;
    alpha_=1;
end

tbot.resetPose();
pause(1)
fprintf("All goals reached!\n");
tbot.resetPose();

function plot_pose(x, y, theta, goal_pose, x_, y_,k_rho,k_alpha,k_beta)
    figure(1); clf; hold on;            % clear figure, hold plots
    title(['krho:',num2str(k_rho),' kalpha:',num2str(k_alpha),' kbeta:',num2str(k_beta)])
    plot(x, y,'--or', 'MarkerSize', 10)  % display (x,y) location of the robot
    plot(goal_pose(1), goal_pose(2),'bx', 'MarkerSize', 15)
    
    quiver(x,y,2*cos(theta),2*sin(theta), 0.1, 'Color','r','LineWidth',1, ...
        'ShowArrowHead',1)

    quiver(goal_pose(1),goal_pose(2),cos(goal_pose(3)),sin(goal_pose(3)), 0.1, ...
        'Color','b','LineWidth',1, 'ShowArrowHead',1)

    plot(x_(1:5:end), y_(1:5:end),'--','Color','b')
    quiver(0,0,1,0,'r')                 % draw arrow for x-axis 
    quiver(0,0,0,1,'g')                 % draw arrow for y-axis 
    axis([-2, 2, -2, 2])                % the limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')



    pause(0.1)
end

function [rho_new, alpha_new, beta_new] = update_parameters(rho, alpha, beta, k_rho, k_alpha, k_beta,T)
    dRho = -k_rho*rho*cos(alpha);
    dAlpha = k_rho*sin(alpha)-k_alpha*alpha-k_beta*beta;
    dBeta = -k_rho*sin(alpha);

    rho_new = dRho*toc(T)+rho;
    alpha_new = dAlpha*toc(T)+alpha;
    alpha_new = atan2(sin(alpha_new),cos(alpha_new));
    beta_new = dBeta*toc(T)+beta;
    beta_new = atan2(sin(beta_new),cos(beta_new));
end