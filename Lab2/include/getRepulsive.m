function [fox,foy] = getRepulsive(tbot,m, x_active, y_active)
%GETREPULSIVE gets the repulsive force for VFF
Fcr=-0.2; 
[x1, y1, ~] = tbot.readPose();
%x1 = x1/20;
%y1 = y1/20;
N=15;

% %Limits for the active window
% lowLimitX = ceil(max([x1-N 1]));
% highLimitX = ceil(min([x1+N size(m,1)]));
% lowLimitY = ceil(max([y1-N 1]));
% highLimitY = ceil(min([y1+N size(m,2)]));
% 
% %Finds the non zero values
% [y,x] = find(m);
% 
% %Gets the points in the active window
% y_active=y(x>lowLimitX & x<highLimitX & y>lowLimitY & y<highLimitY);
% x_active=x(x>lowLimitX & x<highLimitX & y>lowLimitY & y<highLimitY);

fx=zeros(1,size(x_active,1));
fy=zeros(1,size(y_active,1));

for i=1:size(x_active,1)
    %cellValue = m(y_active(i),x_active(i));
    verx=(x_active(i)-x1)/(sqrt((x_active(i)-x1)^2+(y_active(i)-y1)^2));
    very=(y_active(i)-y1)/(sqrt((x_active(i)-x1)^2+(y_active(i)-y1)^2));

    fx(i)=(Fcr*x_active(i)*verx)/(sqrt((x_active(i)-x1)^2+(y_active(i)-y1)^2));
    fy(i)=(Fcr*y_active(i)*very)/(sqrt((x_active(i)-x1)^2+(y_active(i)-y1)^2));
end

fox=sum(fx);
foy=sum(fy);
end
