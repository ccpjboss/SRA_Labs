function plotPose(x, y, theta,x_,y_,map, goalPose, n_points, nPose)
    figure(1); clf; hold on; % clear figure, hold plots
    plot(x, y,'--or', 'MarkerSize', 10)  % display (x,y) location of the robot
    plot(x_(1:2:end), y_(1:2:end),'--')
    quiver(x,y,cos(theta),sin(theta), 0.1, 'Color','r','LineWidth',1, 'ShowArrowHead',1)
    quiver(x,y,cos(nPose(3)),sin(nPose(3)), 0.1, 'Color','b','LineWidth',1, 'ShowArrowHead',1)
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


    for i = 2:79
        for j = 2:79
            if map(i,j) > 0.6
                %scatter(j/20,i/20,50,"black","filled")
                plot(j./20,i./20, '--', 'LineWidth', 2, 'Color', 'black', 'Marker','.')
            end
        end
    end

end


