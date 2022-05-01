
%
% Draws (2D) TurtleBot Robot at (x,y,theta)
%
%  - (x,y) - 2d location [meters] (scalars)
%  - (theta) - orientation angle [radians] (scalar)
% Returns the plot handle (hp) for all displayed parts.
% 
% Tip: delete(hp); clears the robot plot.     
%  


function [hp] = drawTurtleBot(x,y,theta)
    
    % plot handler (6 plot parts)
    hp = zeros(6,1);  

    % circumference slice resolution 
    t =[0:pi/6:2*pi]; 

    % In contour control points (inner radius = 8cm)
    TInData = [0.08 * cos(t) + x; 0.08 * sin(t) + y];

    % Out contour control points (outer radius = 10.5cm)
    TOutData = [ 0.105 * cos(t) + x; 0.105 * sin(t) + y]; 

    % base wheel data
    WheelData = [0.033, 0.033, -0.033, -0.033, 0.033; 0.01, -0.01, -0.01, 0.01, 0.01; ones(1, 5)];
    RWheelData = [1, 0, 0; 0, 1, -0.08; 0, 0, 1] * WheelData;
    LWheelData = [1, 0, 0; 0, 1, 0.08; 0, 0, 1] * WheelData;

    % apply pose transformation
    TRWheelData = [cos(theta), -sin(theta) x; sin(theta), cos(theta) y; 0, 0, 1] * RWheelData;
    TLWheelData = [cos(theta), -sin(theta) x; sin(theta), cos(theta) y; 0, 0, 1] * LWheelData;


    % Lidar (w/ 0.031 radius and 0.305 offset)
    LData= [ 0.031 * cos(t) - 0.0305; 0.031 * sin(t); ones( 1, length(t)) ]; 
    % apply pose transformation
    TLData = [cos(theta), -sin(theta) x; sin(theta), cos(theta) y; 0, 0, 1] * LData;



    % ------------------ 
    % Draw Robot
    % ------------------ 

    % enable plot hold, if disabled 
    if (~ishold) hold on; end 

    % draw out circumferences
    hp(1) = plot(TInData(1,:), TInData(2,:),'k');
    hp(2) = plot(TOutData(1,:), TOutData(2,:),'k');

    % draw lidar
    hp(3) =plot(TLData(1,:), TLData(2,:),'k');

    % draw wheels
    hp(4) = fill(TRWheelData(1,:), TRWheelData(2,:),'k');
    hp(5) = fill(TLWheelData(1,:), TLWheelData(2,:),'k');

    % draw arrow (oriented by theta)
    hp(6) =quiver(x, y, 0.15*cos(theta), 0.15*sin(theta),'k'); 
    







    % show robot at reference frame (DEBUG)
    if(0)

        figure(2); clf; 
        % draw out circumferences
        plot(TInData(1,:) - x, TInData(2,:) - y,'k'); hold on; 
        plot(TOutData(1,:) -x, TOutData(2,:) - y,'k');

        % draw lidar
        plot(LData(1,:), LData(2,:),'k');

        % draw wheels
        fill(RWheelData(1,:), RWheelData(2,:),'k');
        fill(LWheelData(1,:), LWheelData(2,:),'k');
        
        % draw arrow
        quiver(0, 0, 0.15, 0,'k'); 
        title('TurtleBot at base reference')
        pause(0.2);

    end 


