close all
clearvars -except tbot
addpath include/

% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end 
tbot.setPose(3.5,3.5,pi/2);

map = read_map("./maps/fmap_grid5.png");
[x,y,theta] = tbot.readPose();
plotPose(x,y,theta,map);

active_cells = getActiveArea([x,y],map,30);