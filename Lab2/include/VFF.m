function [frx,fry,fox,foy] = VFF(tbot,m,goalPose)
%VFF Calculates the resultant forces for VFF
%   Detailed explanation goes here

% Gets the atractive and the repulsive forces
[fax,fay] = getAtract(tbot,goalPose);
[fox,foy] = getRepulsive(tbot,m);

% Sums the forces
frx=fax+fox;
fry=fay+foy;
end