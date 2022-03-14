

%
% TurtleBot control class 
%
%       Autonomous Robotic Systems 2022, DEEC, UC 
%
% versions:
% v01 - initial release 
% v02 - added setPose() and resetPose() functions [only work in gazebo], angles are now in radians



classdef TurtleBot < handle

properties (Access = private)
    velPub      % robot's velocity publisher 
    velMsg      % velocity message
    odomSub     % odometry subscriber 
    lidarSub    % lidar subscriber 
    gzPub       % gazebo set model state publisher
    gzStateMsg  % gazebo state message 
    gzBotname   % robot name in gazebo
    end

   methods

    % Constructor (two options:)  
    % obj = TurtleBot();
    % obj = TurtleBot(IP_TURTLEBOT, IP_HOST_COMPUTER);
    function obj = TurtleBot( varargin )

        % check input args 
        if (nargin == 0)        % no input args
            % -> REQUIRES MANUALLY SETTING THE IP ADDRESSES 
            IP_TURTLEBOT = "172.16.218.129";              % VIRTUAL MACHINE IP 
%             IP_TURTLEBOT = "10.206.7.1";                % TURTLE ROBOT IP 
            IP_HOST_COMPUTER = "192.168.1.74";          % LOCAL IP

        elseif (nargin ==2 )    % two input args
            IP_TURTLEBOT = varargin{1};                 % Assign 1st input arg
            IP_HOST_COMPUTER = varargin{2};             % Assign 2nd input arg
        else                    % invalid input args
            error("Usage: obj = TurtleBot(IP_TURTLEBOT, IP_HOST_COMPUTER)");
        end 
        
        % Init ROS 
        rosinit(IP_TURTLEBOT, "NodeHost", IP_HOST_COMPUTER);

        % create a publisher for the /cmd_vel topic
        obj.velPub = rospublisher("/cmd_vel","DataFormat","struct");
        % create a velocity message 
        obj.velMsg = rosmessage(obj.velPub);


        %subscriber for the odometry messages
        obj.odomSub = rossubscriber("/odom","DataFormat","struct");

        %subscribe to the lidar topic:
        obj.lidarSub = rossubscriber("/scan","DataFormat","struct");


        % -> Only initialize gazebo model state topic in simulation mode 
        if (IP_TURTLEBOT ~= "10.206.7.1")
            % query gazebo's objects: 
            % subscriber for the model_states messages
            gzSub = rossubscriber("/gazebo/model_states","DataFormat","struct");
            gzMsg = receive(gzSub,3); 
            % get gazebo's turtlebot name (last item)
            obj.gzBotname = gzMsg.Name{end};

            % create a publisher for the /gazebo/set_model_state topic
            obj.gzPub = rospublisher("/gazebo/set_model_state","DataFormat","struct");
            % create a gz state message 
            obj.gzStateMsg = rosmessage(obj.gzPub); 
        end 

    end

    % Send linear and angular velocity commands. 
    % (v,w) are scalars with units in [m/s] and [rads/s], respectively  
    function setVelocity(obj, v, w)
        obj.velMsg.Linear.X = v(1);
        obj.velMsg.Linear.Y = 0;
        obj.velMsg.Linear.Z = 0;
        obj.velMsg.Angular.X = 0;
        obj.velMsg.Angular.Y = 0;
        obj.velMsg.Angular.Z = w(1);
        % send velocity message
        send(obj.velPub, obj.velMsg)
    end

    % Stop robot - send (zero) velocity commands
    function stop(obj)
        obj.setVelocity( 0, 0)
    end


    % Reads 2D pose 
    % return (x,y) positon [in meters] and (theta) orientation [rad]
    function [x,y,theta] = readPose(obj)
        odomMsg = receive(obj.odomSub,3);
        pose = odomMsg.Pose.Pose;
        x = pose.Position.X;
        y = pose.Position.Y;
        %z = pose.Position.Z;
        
        % read pose (returns a quaternion)
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);      
        % angles in Euler format: ZYX
        %theta = rad2deg( angles(1) ); 
        theta = angles(1);                      % return theta in radians 
    end


    % Set 2D pose to (x,y,theta)
    % note: only works in gazebo
    function setPose(obj, x, y, theta)
        %obj.gzStateMsg.ModelName = 'turtlebot3_burger';    % empty world
        %obj.gzStateMsg.ModelName = 'turtlebot3';           % house world
        obj.gzStateMsg.ModelName = obj.gzBotname;           % generic ? 
        % set position 
        obj.gzStateMsg.Pose.Position.X = x;
        obj.gzStateMsg.Pose.Position.Y = y;
        %obj.gzStateMsg.Pose.Position.Z = 0;
        
        % convert Euler angles (ZYX) to quaternion qt=[W,X,Y,Z]
        qt = eul2quat( [theta, 0, 0] );
        % set orientation 
        obj.gzStateMsg.Pose.Orientation.X = qt(2);
        obj.gzStateMsg.Pose.Orientation.Y = qt(3);
        obj.gzStateMsg.Pose.Orientation.Z = qt(4);
        obj.gzStateMsg.Pose.Orientation.W = qt(1);

        % send st state message
        send(obj.gzPub, obj.gzStateMsg);
    end 


    % Reset turtlebot's pose to origin: (x,y,theta) = (0,0,0) 
    % note: only works in gazebo
    function resetPose(obj)
        obj.stop()                      % stop robot
        pause(2)
        obj.setPose(0.0, 0.0, 0.0);     % set pose to origin (forward oriented to x axis)  
    end 


    % Read lidar data, returns scanMsg structure 
    % - use rosPlot(scanMsg) to display
    function scanMsg = readLidar(obj)
        scanMsg = receive(obj.lidarSub);
        
        % convert laser scan readings to Cartesian coordinates
        %[cart, angles] = rosReadCartesian(scan);
    end 


    % Destructor: clear the workspace of publishers, subscribers, and other ROS-related objects
    % note: stops robot and disconnects ROS, if class object if deleted.   
    function delete(obj)
        obj.stop()      % stop robot 
        rosshutdown     % shutdown ROS
    end

    end 
end


