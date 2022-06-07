
% 
% Demo - TurtleBot3 Keyboard Speed Control 
%
% Run either in Gazebo or with the real TurtleBot robot;
%
% [Gazebo]
% Launch the virtual machine.
% Open terminal, and start the empty Gazebo world: ./start-gazebo-empty.sh.  
%
% [Real TurtleBot]
% Connect directly to TurtleBot using IP_TURTLEBOT = "10.206.7.1";  
%
% Versions: 
% v01 - initial version
%

% clear memory 
clearvars -except tbot

% start TurtleBot connection (tbot object), only if required
if ( ~exist("tbot") ) 
    % init by assign the IPs directly by the TurtleBot constructor
    IP_TURTLEBOT = "192.168.1.110";         % virtual machine IP (or robot IP)
    %IP_TURTLEBOT = "10.206.7.1";           % real TurtleBot robot IP
    IP_HOST_COMPUTER = "192.168.1.100";     % local machine IP
    tbot = TurtleBot(IP_TURTLEBOT, IP_HOST_COMPUTER);
    % check version
    if( tbot.getVersion() < 0.6 ) error('TurtleBot v06 required'); end 
end 


% print connection type 
if( tbot.isRealRobot() )
    fprintf('Connected to real TurtleBot\n');
else
    fprintf('Connected to Gazebo simulator\n');
end

% print battery level
fprintf('TurtleBot battery level = %g\n', tbot.getBatteryLevel()*100)

% print keyboard instructions
fprintf('Keyboard keys:\n [<-] turn left (+ angular speed)\n [->] turn right\n [up] + linear speed\n [down] - linear speed\n [space] stop\n [esc] exit\n');

% reset turtlebot's 2D pose (x,y,theta) -> (0,0,0) 
tbot.resetPose();



% define global variables (speeds + exit flag) 
% [used to share data berween main script and local function]
global v;
global w;
global exitFlag;

% init linear + angular velocities
v = 0;
w = 0;  

% init exit flag
exitFlag = false;

% init figure w/ keyboard handler function 
h = figure('KeyPressFcn', @keyboardHandle);

% init ratecontrol obj (run at 5Hz)
r = rateControl(5);     


while(1)

    % check for termination 
    if( exitFlag ) break; end 
 
    % read TurtleBot pose
    [x, y, theta, timestamp] = tbot.readPose();

    figure(h);                          % select figure (h) 
    clf; hold on;                       % clear figure, hold plots
    drawTurtleBot(x, y, theta);         % draw Robot
    quiver(0,0,0.5,0,'r')               % draw arrow for x-axis 
    quiver(0,0,0,0.5,'g')               % draw arrow for y-axis 
    axis([-2.5, 2.5, -2.5, 2.5])        % limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')
    title( sprintf('TurtleBot (v,w) = (%1.3f,%1.3f)', v, w) )

    waitfor(r);                         % adaptive pause

    % send linear + angular velocities command
    tbot.setVelocity(v,w); 

end 


% close window
close(h); 

% stop robot 
tbot.stop();




% keyboard handler function
function keyboardHandle(hObject, eventData)
    % global variables
    global v;
    global w;
    global exitFlag;

    % set keyboard keys update values
    linearUpdate = 0.025;
    angularUpdate = 0.05;

    switch eventData.Key
        case 'rightarrow'
            w = w - angularUpdate; 
        case 'leftarrow'
            w = w + angularUpdate; 
        case 'uparrow'
            v = v + linearUpdate;
        case 'downarrow'
            v = v - linearUpdate;
        case 'space'
            v = 0; 
            w = 0;
        case 'escape'
            fprintf('terminate\n');
            v = 0; 
            w = 0;
            exitFlag = true;
        otherwise 
            fprintf('invalid key\n'); 
    end 

end     % end nested local function 


