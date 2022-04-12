function [fox,foy] = getRepulsive(tbot, x_active, y_active)
%GETREPULSIVE gets the repulsive force for VFF
Fcr=-0.5; 
[x1, y1, ~] = tbot.readPose();

fx=zeros(1,size(x_active,1));
fy=zeros(1,size(y_active,1));

for i=1:size(x_active,1)
    verx=(x_active(i)-x1)/(sqrt((x_active(i)-x1)^2+(y_active(i)-y1)^2));
    very=(y_active(i)-y1)/(sqrt((x_active(i)-x1)^2+(y_active(i)-y1)^2));

    fx(i)=(Fcr*1*verx)/(sqrt((x_active(i)-x1)^2+(y_active(i)-y1)^2));
    fy(i)=(Fcr*1*very)/(sqrt((x_active(i)-x1)^2+(y_active(i)-y1)^2));
end

fox=sum(fx);
foy=sum(fy);

end
