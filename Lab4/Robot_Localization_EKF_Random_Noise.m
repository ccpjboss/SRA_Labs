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
handle = [];
tbot.setPose(2, 1, 0);                       % set initial pose (don't use resetPose(.) in scmap)
u = [0.1; 0.1];                              % circle R=1 around the central square
pause(2);
tbot.setVelocity(u(1), u(2));                % set velocities
[x, y, theta, ptimestamp] = tbot.readPose(); % read pose once (required to startup the encoder data)
r = rateControl(5);                          % init ratecontrol obj (loop at 5Hz)

dead_reckoning = [2.0; 1.0; 0];
not_update = dead_reckoning;
ground_truth = [2.0; 1.0; 0];
b = tbot.getWheelBaseline();

C_p = diag([0.09 0.09 deg2rad(1)]).^2; % 0.01 (meters) and 1 degree

lidar_max_dist = 2; % 2 meters max Lidar reading

gt = [];
ekf = [];
predit = [];

tic;
while (toc < 67) % run for a given time (s)

    v_valid = [];
    valid_J_g = [];
    R = [];
    s = [];

    %% Prediction
    [dsr, dsl, pose2D, timestamp] = tbot.readEncodersDataWithNoise([0.009, 0.009]); % read data from encoders

    ground_truth = pose2D;
    [scanMsg, lddata, ldtimestamp] = tbot.readLidar();

    C_u = diag([0.0005*dsr 0.0005*dsl]);

    dead_reckoning = getDeadReckoning(dead_reckoning, dsr, dsl, b+0.03); % eq 7
    not_update = getDeadReckoning(not_update, dsr, dsl, b+0.03);
    
    [xEst_cell, yEst_cell] = world2grid(dead_reckoning(1),dead_reckoning(2),80);
    
    %clip the values
    xEst_cell = min(max(xEst_cell,1),80);
    yEst_cell  = min(max(yEst_cell,1),80);

    C_p = getMotionError(C_p,C_u,dead_reckoning, dsr, dsl, b+0.03); % eq 8

    %% Observation
    lidar_angles = linspace(0,2*pi,360); % Get lidar angles with alpha = 1 degree
    xy = [lidar_max_dist*cos(lidar_angles); lidar_max_dist*sin(lidar_angles)]; % Get xy from distant cells
    
    transformMatrix = [cos(dead_reckoning(3)) -sin(dead_reckoning(3)) dead_reckoning(1)-0.0305*cos(dead_reckoning(3)); % Transformation Lidar -> World
        sin(dead_reckoning(3)) cos(dead_reckoning(3)) dead_reckoning(2)-0.0305*sin(dead_reckoning(3));
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
    J_g = [];

    invalid = 1;
    for i=1:size(cell_x,1)
        [xl, yl] = bresenham(xEst_cell,yEst_cell,cell_x(i),cell_y(i));
        for j = 1:size(xl,1)
            if (map(yl(j),xl(j)) == 1)
                invalid = 0;
                
                [x_aux, y_aux] = grid2world(xl(j),yl(j), 80);
                x_obs = [x_obs;x_aux];
                y_obs = [y_obs;y_aux];

                g = sqrt((dead_reckoning(1)-x_aux)^2+(dead_reckoning(2)-y_aux)^2);
                g_est = [g_est;g];
                J_g(:,:,i) = getJacobianObs(dead_reckoning,x_aux,y_aux);
                break
            end
        end
        if invalid == 1
            g_est = [g_est;nan];
        end
        invalid = 1;
    end

    lidar_test = transformMatrix*[lddata.Cartesian';ones(1,size(lddata.Cartesian,1))];
    lidar_test = [lidar_test(1,:)',lidar_test(2,:)'];
    g_true = lddata.Ranges;
    v = g_true - g_est;
    [row,~] = find(~(isnan(v)));

    e = 1;
    for i = 1:size(row,1)
        idx = row(i);
        R_aux = 0.035*g_true(idx);
        s = J_g(:,:,idx)*C_p*J_g(:,:,idx)'+R_aux; % eq 12
        e_gait = v(idx)*(1/s)*v(idx)';
        if (e_gait < e^2)                    % eq 14
            v_valid(i)=v(idx);               % eq 15
            valid_J_g(i,:) = J_g(:,:,idx);   % eq 16
            R(i) = R_aux;
        end
    end
    
    inov_cov = valid_J_g*C_p*valid_J_g'+diag(R); % eq 17
    Kalman_gain = C_p*valid_J_g'*pinv(inov_cov); % eq 18

    dead_reckoning = dead_reckoning + Kalman_gain*v_valid';
    C_p = C_p-Kalman_gain*inov_cov*Kalman_gain';

    % Display
    figure(1)
    axis([0 4 0 4])
    grid on;
    gt = [gt;ground_truth(1) ground_truth(2)];
    predit = [predit;not_update(1) not_update(2)];
    ekf = [ekf;dead_reckoning(1) dead_reckoning(2)];
    
    plot(not_update(1), not_update(2), '.m');
    plot(ground_truth(1), ground_truth(2), '.b');
    plot(dead_reckoning(1), dead_reckoning(2), '.g');
    legend('Map','Not Updated','Ground Truth', 'EKF')
    waitfor(r);
end

figure("Name",'Final plot of the simulation');
plot(map_y./20,map_x./20,'.','Color','black'); hold on
grid minor
plot(gt(:,1),gt(:,2),'.-b');
plot(ekf(:,1),ekf(:,2),'.-g');
plot(predit(:,1),predit(:,2),'.-m');
legend('Map','Gazebo','EKF', 'Encoders')
title('Final plot')

ssd = sum((gt(:,1)-ekf(:,1)).^2+(gt(:,2)-ekf(:,2)).^2)
mean_ssd = ssd/size(ekf,1)

tbot.stop(); % stop robot
