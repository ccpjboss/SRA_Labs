function plotPose(x, y, theta,x_,y_,map, goalPose, n_points)

    figure(1); 
    plot(x_(1:2:end), y_(1:2:end),'--')
    drawTurtleBot(x,y,theta);
    quiver(0,0,1,0,'r')                 % draw arrow for x-axis 
    quiver(0,0,0,1,'g')                 % draw arrow for y-axis 
    axis([0, 4, 0, 4])                % the limits for the current axes [xmin xmax ymin ymax]
    grid on;
    grid minor;% enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')

    for j = 1:n_points
        if (j == n_points)
            plot(goalPose(j,1),goalPose(j,2),'gx', 'MarkerSize', 5); %display locations of points
        else
            plot(goalPose(j,1),goalPose(j,2),'bx', 'MarkerSize', 5); %display locations of points
        end
    end
    
    [r,c] = find(map>0.75);
    scatter(c./(size(map,1)/4),r./(size(map,1)/4),[],[0 0 0],'Marker','.')

end


