function [world_x, world_y] = updatemap(tbot, map, x, y, theta)

    [scanMsg, xydata, angles] = tbot.readLidar();

    
    wt = [cos(theta) -sin(theta) x - 0.0305; 
          sin(theta)  cos(theta) y;
          0           0          1];

    world = wt .* xydata;
    figure();
    hold on
    for i = 1:size(xydata)
        plot(xydata(i,1), xydata(i,2), '+', 'color', 'r')
        plot(world(i,1), world(i,2), '+', 'color', 'r')
        %world_x = wt * xydata(i,1);
        %world_y = wt * xydata(i,2);
        %plot(world_x(i,1), world_y(i,2), '+','color', 'r')
    end

end