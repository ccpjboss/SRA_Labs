clearvars -except tbot

if ( ~exist("tbot") ) 
    tbot = TurtleBot();     
end 
addpath("include/")
tbot.setPose(0.5,0.5,0);
tbot.stop()
map = zeros(80);
map(:) = 0.5;
l0 = zeros(80);
l0(1,:) = 0.2;
l0(end,:) = 0.2;
l0(:,1) = 0.2;
l0(:,end) = 0.2;

while (1)             
    [scan, xydata, angles] = tbot.readLidar();
    [x,y,theta] = tbot.readPose();
    [robotX, robotY] = world2grid(x,y,80);

    transformMatrix = [cos(theta) -sin(theta) x-0.0305;
    			sin(theta) cos(theta) y;
			0 0 1];

    xyWorld = transformMatrix*[xydata';ones(1,size(xydata,1))];
    xyWorld = xyWorld(1:2,:);
    [xCell, yCell] = world2grid(xyWorld(1,:),xyWorld(2,:),80);
    map = updateMap(map,xCell,yCell,[robotX, robotY],l0);

    figure(1);
    subplot(2,2,1);
    rosPlot(scan);
    subplot(2,2,2);
    plot(xyWorld(1,:)',xyWorld(2,:)');
    title('Laser Scan in World Frame')
    axis([0 4 0 4]);
    grid minor;
    subplot(2,2,3);
    plot(xCell,yCell);
    title('Discrete map')
    axis([0 80 0 80]);
    grid minor;
    subplot(2,2,4);
    imshow(rot90(imcomplement(map)),[])
    title('map')

    tbot.setVelocity(0.075,0);
end
