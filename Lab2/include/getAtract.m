function [fax,fay] = getAtract(tbot,goalPose)
%GETATRACT gets the atractive force for VFF

Fca=25; %Constant atractive force
[x, y, ~] = tbot.readPose();

verx=(goalPose(1)-x)/(sqrt((goalPose(1)-x)^2+(goalPose(2)-y)^2));
very=(goalPose(2)-y)/(sqrt((goalPose(1)-x)^2+(goalPose(2)-y)^2));

fax=Fca*verx;
fay=Fca*very;

end
