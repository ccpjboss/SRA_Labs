
% 
% Demo - Read 2D pose from TurtleBot3 (pooling vs ~fixed timming) 
%
% Launch the virtual machine.
% Open terminal, and start the empty Gazebo world: ./start-gazebo-empty.sh.
% [Gazebo tip] The robot initial pose configuration can be recovered at Edit -> Reset Model Poses
% versions:
% v01 - initial release 
% v02 - updated w/ resetPose()
% v03 - tested w/ TurtleBot object v03

% clear memory 
clearvars -except tbot

% init TurtleBot connection (tbot object), only if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot();     
    % Alternative -> assign the IPs directly by the TurtleBot constructor  
    % tbot = TurtleBot(IP_TURTLEBOT, IP_HOST_COMPUTER);
end 

% reset turtlebot's 2D pose (x,y,theta) -> (0,0,0) 
tbot.resetPose();


% start by moving the robot in a circular motion 
tbot.setVelocity(0.1, 0.25)


tic
% run from 10 seconds
while (toc < 10)
    
    % read TurtleBot pose 
    [x,y,theta] = tbot.readPose();

    figure(1); clf; hold on;            % clear figure, hold plots
    plot(x, y,'ko', 'MarkerSize', 10)   % display (x,y) location of the robot
    quiver(x, y, 0.25*cos(theta), 0.25*sin(theta),'k')   % draw arrow (oriented by theta) 
    quiver(0,0,1,0,'r')                 % draw arrow for x-axis 
    quiver(0,0,0,1,'g')                 % draw arrow for y-axis 
    axis([-3, 3, -3, 3])                % the limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')
    pause(0.1)

end

% stop robot 
tbot.stop()





% ---------------------------------------------------------
% now, replace the previous code by an controlled time loop
% ---------------------------------------------------------

% % init ratecontrol obj (enables to run a loop at a fixed frequency) 
% r = rateControl(5);     % run at 5Hz

% % run 50 iterations (50 x 1/5 = 10s) 
% for i = 1:1:50

%     % read TurtleBot pose 
%     [x,y,theta] = tbot.readPose();

%     figure(1); clf; hold on;            % clear figure, hold plots
%     plot(x, y,'ro', 'MarkerSize', 10)   % display (x,y) location of the robot
%     quiver(x, y, 0.25*cos(theta), 0.25*sin(theta),'k')   % draw arrow (oriented by theta) 
%     quiver(0,0,1,0,'r')                 % draw arrow for x-axis 
%     quiver(0,0,0,1,'g')                 % draw arrow for y-axis 
%     axis([-3, 3, -3, 3])                % the limits for the current axes [xmin xmax ymin ymax]
%     grid on;                            % enable grid 
%     xlabel('x')                         % axis labels 
%     ylabel('y')
  
%     % adaptive pause
%     waitfor(r);
% end

% % check statistics of past execution periods
% stats = statistics(r);

% % stop robot 
% tbot.stop()
 