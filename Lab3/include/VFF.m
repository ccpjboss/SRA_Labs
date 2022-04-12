function [frx,fry,fox,foy] = VFF(tbot,xyWorld,goalPose)
%VFF Calculates the resultant forces for VFF
%   Detailed explanation goes here

active_cells = [];
c = 0;

[x,y,theta] = tbot.readPose();

for i=1:size(xyWorld,1)
    dist = sqrt((xyWorld(i,1)-x)^2+(xyWorld(i,2)-y)^2);
    
    if dist < 0.5
        active_cells(i,:) = xyWorld(i,:);
        c = c + 1;
    end
end

if c == 0
    % Gets the atractive and the repulsive forces
    [fax,fay] = getAtract(tbot,goalPose);
    fox = 0;
    foy = 0;

else   
    % Gets the atractive and the repulsive forces
    [fax,fay] = getAtract(tbot,goalPose);
    [fox,foy] = getRepulsive(tbot,active_cells(:,1), active_cells(:,2));
end
            
% Sums the forces
frx=fax+fox;
fry=fay+foy;
end