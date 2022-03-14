function [fox,foy] = getRepulsive(tbot,m)
%GETREPULSIVE gets the repulsive force for VFF
Fcr=-2; 
[x, y, ~] = tbot.readPose();
x = x * 100;
y = y * 100;
N=15;

%Limits for the active window
lowLimitX = ceil(max([x-N 1]));
highLimitX = ceil(min([x+N size(m,1)]));
lowLimitY = ceil(max([y-N 1]));
highLimitY = ceil(min([y+N size(m,2)]));

%Finds the non zero values
[y,x] = find(m);

%Gets the points in the active window
y_active=y(x>lowLimitX & x<highLimitX & y>lowLimitY & y<highLimitY);
x_active=x(x>lowLimitX & x<highLimitX & y>lowLimitY & y<highLimitY);

fx=zeros(1,size(x_active,1));
fy=zeros(1,size(y_active,1));

for i=1:size(x_active,1)
    cellValue = m(y_active(i),x_active(i));
    verx=(x_active(i)-x)/(sqrt((x_active(i)-x)^2+(y_active(i)-y)^2));
    very=(y_active(i)-y)/(sqrt((x_active(i)-x)^2+(y_active(i)-y)^2));

    fx(i)=(Fcr*cellValue*verx)/(sqrt((x_active(i)-x)^2+(y_active(i)-y)^2));
    fy(i)=(Fcr*cellValue*very)/(sqrt((x_active(i)-x)^2+(y_active(i)-y)^2));
end

fox=sum(fx);
foy=sum(fy);
end
