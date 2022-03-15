close all
clearvars -except tbot

% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end 
tbot.setPose(1,0.5,pi/2);

map = read_map("./maps/fmap_grid5.png");
[x,y,theta] = tbot.readPose();
plotPose(x,y,theta,map);

activeArea = getActiveArea([x,y],map,20);