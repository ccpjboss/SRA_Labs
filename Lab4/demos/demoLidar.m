
% 
% Demo - Read from TurtleBot3 Lidar
%
% Launch the virtual machine.
% Open terminal, and start the house Gazebo world: ./start-gazebo-house.sh.
%
% Versions:
% v01 - initial release 
% v02 - updated w/ setPose()
% v03 - updated readLidar() function
% v04 - updated readLidar() function, added data selection, timeDiff computation, removed pooling demo.
% v05 - tested w/ TurtleBot class v05


% clear memory 
clearvars -except tbot

% start TurtleBot connection (tbot object), only if required
if ( ~exist("tbot") ) 
    % init by assign the IPs directly by the TurtleBot constructor
    IP_TURTLEBOT = "192.168.1.110";         % virtual machine IP (or robot IP)
    IP_HOST_COMPUTER = "192.168.1.100";     % local machine IP
    tbot = TurtleBot(IP_TURTLEBOT, IP_HOST_COMPUTER);
    % alternative -> edit TurtleBot.m and define the robot and local host IP addresses
    %tbot = TurtleBot();
    % check version
    if( tbot.getVersion() < 0.5 ) error('TurtleBot v05 is required'); end 
end 


% set turtlebot's 2D pose (x,y,theta) -> (-3,1,0): [default for this world]
tbot.setPose(-3,1,0);

% start by moving the robot forward
tbot.setVelocity(0.075, 0)


% -------------------------------------
%  read lidar data w/ fixed timming
% -------------------------------------

% init ratecontrol obj (enables to run a loop at a fixed frequency) 
r = rateControl(5);     % run at 5Hz

% run for 15 seconds: 75 iterations (75 x 1/5 = 15s) 
for i = 1:1:75

    % read TurtleBot pose 
    [x, y, theta, timestamp] = tbot.readPose();

    % load lidar data from robot 
    [scan, lddata, ldtimestamp] = tbot.readLidar();

    % compute the time diference between the pose reading and laser scan (in seconds) 
    timeDiff = ldtimestamp - timestamp;  


    % Get in range lidar data indexs (valid data - obstacles).
    [vidx] = tbot.getInRangeLidarDataIdx(lddata);
    % load valid X/Y cartesian lidar readings          
    xydata = lddata.Cartesian(vidx,:);


    % Get out of range lidar data indexs (too near or too far readings).
    [nidx] = tbot.getOutRangeLidarDataIdx(lddata);
    % load obstacle free orientations (angles)  
    freeAng = lddata.Angles(nidx);




    % display data (using ROS toolbox)
    figure(1); clf;
    rosPlot( scan ) 

    % display data (using regular plots)
    figure(2); clf;
    plot(xydata(:,1), xydata(:,2),'b.','MarkerSize',8)   % draw laser readings
    axis([-4, 4, -4, 4])                % the limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')

    % adaptive pause
    waitfor(r);
end

% check statistics of past execution periods
stats = statistics(r);

% stop robot 
tbot.stop()








% % -----------------------------------------------
% %  read lidar data by pooling
% % -----------------------------------------------
% fprintf('read lidar data by pooling\n')

% tic
% % run from 10 seconds
% while (toc < 10)             
%
%     % load lidar data from robot 
%     [scan, lddata, ldtimestamp] = tbot.readLidar();
%
%     % display data
%     figure(1); clf;
%     rosPlot( scan );
%     pause(0.1);
% end

% % stop robot 
% tbot.stop()

