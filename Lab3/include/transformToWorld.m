function [xyWorld] = transformToWorld(xydata,x,y,theta)
    transformMatrix = [cos(theta) -sin(theta) x-0.0305;
    			sin(theta) cos(theta) y;
			0 0 1];

    xyWorld = transformMatrix*[xydata';ones(1,size(xydata,1))];
    xyWorld = [xyWorld(1,:)',xyWorld(2,:)'];
end
