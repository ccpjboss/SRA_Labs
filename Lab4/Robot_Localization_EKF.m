clearvars -except tbot
close all
addpath include /

if (~exist("tbot", 'var'))
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot();
end

map = readMap('maps/scmap_grid5.png');
figure(1);
[map_y,map_x] = find(map);
plot(map_y./20,map_x./20,'.','Color','black'); hold on
axis([0, 4, 0, 4])                          % the limits for the current axes [xmin xmax ymin ymax]
grid on;

tbot.setPose(2, 1, 0);                       % set initial pose (don't use resetPose(.) in scmap)
u = [0.1; 0.1];                              % linear velocity and angular velocity
tbot.setVelocity(u(1), u(2));                % set velocities
[x, y, theta, ptimestamp] = tbot.readPose(); % read pose once (required to startup the encoder data)
r = rateControl(1);                          % init ratecontrol obj (loop at 5Hz)
T = r.DesiredPeriod;

dead_reckoning = [2; 1; 0];
ground_truth = [2; 1; 0];
b = tbot.getWheelBaseline();

C_p = diag([0.01 0.01 deg2rad(1)]).^2; % 0.01 (meters) and 1 degree
C_u = [0.05 0; 0 0.07].^2;           % Duvidas (???)

lidar_max_dist = 3; % 2 meters max Lidar reading

tic;
while (toc < 20) % run for a given time (s)
    ground_truth = ground_truth + [u(1) * cos(ground_truth(3)) * T;
        u(1) * sin(ground_truth(3)) * T;
        u(2) * T];

%     [dsr, dsl, pose2D, timestamp] = tbot.readEncodersData(); % read data from encoders

    [dsr, dsl, pose2D, timestamp] = tbot.readEncodersDataWithNoise([0.05; 0.05]); % read data from encoders with Noise

    dead_reckoning = getDeadReckoning(dead_reckoning, dsr, dsl, b);
    [xEst_cell, yEst_cell] = world2grid(dead_reckoning(1),dead_reckoning(2),80);
    
    %clip the values
    xEst_cell = min(max(xEst_cell,1),80);
    yEst_cell  = min(max(yEst_cell,1),80);

    C_p = getMotionError(C_p,C_u,dead_reckoning, dsr, dsl, b);

    lidar_angles = linspace(0,2*pi,360); % Get lidar angles with alpha = 1 degree
    xy = [lidar_max_dist*cos(lidar_angles); lidar_max_dist*sin(lidar_angles)]; % Get xy from distant cells
    
    transformMatrix = [cos(dead_reckoning(3)) -sin(dead_reckoning(3)) dead_reckoning(1)-0.0305; % Transformation Lidar -> World
        sin(dead_reckoning(3)) cos(dead_reckoning(3)) dead_reckoning(2);
        0 0 1];

    xyWorld = transformMatrix*[xy;ones(1,size(xy,2))];
    xyWorld = [xyWorld(1,:)',xyWorld(2,:)'];
    
    [cell_x, cell_y] = world2grid(xyWorld(:,1),xyWorld(:,2),80);

    %clip the values
    cell_x = min(max(cell_x,1),80);
    cell_y  = min(max(cell_y,1),80);
    
    g_est = [];
    x_obs = [];
    y_obs = [];

    invalid = 1;
    for i=1:size(cell_x,1)
        [xl, yl] = bresenham(xEst_cell,yEst_cell,cell_x(i),cell_y(i));
        for j = 1:size(xl,1)
            if (map(yl(j),xl(j)) == 1)
                invalid = 0;
                [x_aux, y_aux] = grid2world(xl(j),yl(j), 80);
                x_obs = [x_obs;x_aux];
                y_obs = [y_obs;y_aux];
                g_est = [g_est;sqrt((dead_reckoning(1)-x_aux)^2+(dead_reckoning(2)-y_aux)^2)];
                break
            end
        end
        if invalid == 1
            g_est = [g_est;nan];
        end
        invalid = 1;
    end

    [scanMsg, lddata, ldtimestamp] = tbot.readLidar();
    lidar_test = transformMatrix*[lddata.Cartesian';ones(1,size(lddata.Cartesian,1))];
    lidar_test = [lidar_test(1,:)',lidar_test(2,:)'];
    g_true = lddata.Ranges;
    v = abs(g_true - g_est);

    % Compute Jacobians of observations (check GridLocalization_EKF pdf)
    J_g = [(1./rmmissing(g_est)).*(dead_reckoning(1) - x_obs);
            1./rmmissing(g_est).*(dead_reckoning(2) - y_obs);
            (x_obs-dead_reckoning(1)).*(0.035*sin(dead_reckoning(3)))+(y_obs-dead_reckoning(2)).*(-0.035*cos(dead_reckoning(3)))];

    
    % Compute the covariance of the inovation
    % s(k+1) = J_g*C_p*J-g'+R(k+1)

    % L&D-W validation
    % v_i*s(k+1)*v_i' <e^2
    % if condition valid -> successful match
    

    % Display
    figure(1)
    axis([0 4 0 4])
    grid on;
    plot(ground_truth(1), ground_truth(2), '.b'); hold on;
    plot(dead_reckoning(1), dead_reckoning(2), '.g'); hold on;
%     showErrorEllipse(dead_reckoning,C_p); 
%     plot(lidar_true(:,1),lidar_true(:,2),'.r')
    plot(x_obs,y_obs,'.');
    legend('Ground Truth', 'Dead Reckoning')
    drawnow

    waitfor(r); % adaptive pause
end

tbot.stop(); % stop robot
