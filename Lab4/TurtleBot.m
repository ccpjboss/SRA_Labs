
%
% TurtleBot control class
%
%       Autonomous Robotic Systems 2022, DEEC, UC
%
% Versions:
% v01 - initial release
% v02 - added setPose() and resetPose() functions [only work in gazebo], angles are now in radians
% v03 - updated setPose() and readLidar() functions. Added velocity saturation values. Added version number.
% v04 - updated readLidar() functionality - included polar data, timestap and data selection. Added timestaps.
% v05 - added (simulated) readEncodersData(), readEncodersDataWithNoise() and getWheelBaseline().
%


classdef TurtleBot < handle

    properties (Access = private)
        version         % version number
        isSimulation    % simulation flag: (1) gazebo (0) real robot
        velPub          % robot's velocity publisher
        velMsg          % velocity message
        odomSub         % odometry subscriber
        lidarSub        % lidar subscriber
        gzPub           % gazebo set model state publisher
        gzStateMsg      % gazebo state message
        gzBotname       % robot name in gazebo
        poset           % 2D pose + time buffer
        wheelbaseline   % wheels separation (distance between wheels)
    end

    methods

        % Constructor (two options:)
        % obj = TurtleBot();
        % obj = TurtleBot(IP_TURTLEBOT, IP_HOST_COMPUTER);
        function obj = TurtleBot( varargin )

            % check input args
            if (nargin == 0)                                % no input args

                % -> REQUIRES MANUALLY SETTING THE IP ADDRESSES
                IP_TURTLEBOT = "172.16.218.129";             % VIRTUAL MACHINE IP
                %IP_TURTLEBOT = "10.206.7.1";               % TURTLE ROBOT IP
                IP_HOST_COMPUTER = "192.168.1.74";         % LOCAL IP

            elseif (nargin ==2 )                            % two input args
                IP_TURTLEBOT = varargin{1};                 % Assign 1st input arg
                IP_HOST_COMPUTER = varargin{2};             % Assign 2nd input arg
            else                                            % invalid input args
                error("Usage: obj = TurtleBot(IP_TURTLEBOT, IP_HOST_COMPUTER)");
            end

            % -------------------
            % Init ROS
            % -------------------
            rosinit(IP_TURTLEBOT, "NodeHost", IP_HOST_COMPUTER);


            % create a publisher for the /cmd_vel topic
            obj.velPub = rospublisher("/cmd_vel","DataFormat","struct");
            % create a velocity message
            obj.velMsg = rosmessage(obj.velPub);


            %subscriber for the odometry messages
            obj.odomSub = rossubscriber("/odom","DataFormat","struct");

            %subscribe to the lidar topic:
            obj.lidarSub = rossubscriber("/scan","DataFormat","struct");



            % Switch between simulation / real robots ### DO NOT CHANGE THESE VALUES
            if ( strcmp( IP_TURTLEBOT, "10.206.7.1") || strcmp( IP_TURTLEBOT, "10.206.7.2") )
                obj.isSimulation = 0;           % clear simulation flag (real robot)
            else
                obj.isSimulation = 1;           % set simulation flag (gazebo)
            end

            % only initialize gazebo model state topic in simulation mode
            if ( obj.isSimulation )

                % query gazebo's objects:
                % subscriber for the model_states messages
                gzSub = rossubscriber("/gazebo/model_states","DataFormat","struct");
                % read data from /gazebo/model_states topic
                gzMsg = receive(gzSub,3);


                % list of turtlebot models reference names
                nList = ["turtlebot3_burger", "turtlebot3"];
                % look for reference names
                idx = matches(gzMsg.Name, nList);

                if ( sum(idx) == 1 )
                    % get gazebo's turtlebot name
                    obj.gzBotname = gzMsg.Name{idx};
                    %fprintf("tbot name: %s\n",obj.gzBotname);
                else
                    % turtlebot not found
                    error("TurtleBot not found in Gazeboo world.");
                end

                % create a publisher for the /gazebo/set_model_state topic
                obj.gzPub = rospublisher("/gazebo/set_model_state","DataFormat","struct");
                % create a gz state message
                obj.gzStateMsg = rosmessage(obj.gzPub);

            end


            % init 2D pose (x,y,theta) + time vector
            obj.poset = zeros(4,1);

            % set distance (separation) between wheels (in meters)
            obj.wheelbaseline  = 0.16;

            % set version number
            obj.version = 0.5;

        end


        % get version number
        function vs = getVersion(obj)
            vs = obj.version;
        end


        % get wheel baseline (distance between wheels)
        function b = getWheelBaseline(obj)
            b = obj.wheelbaseline;
        end


        % Send linear and angular velocity commands.
        % (v,w) are scalars with units in [m/s] and [rads/s], respectively
        function setVelocity(obj, v, w)

            % clip (saturate) velocity to maximum values [see turtlebot manual]
            if( abs(v) > 0.22) v = sign(v) * 0.22; end      % [max v = 0.22 m/s]
            if( abs(w) > 2.84) w = sign(w) * 2.84; end      % [max w = 2.84 rad/s]

            % generate ROS velocity message
            obj.velMsg.Linear.X = v(1);
            obj.velMsg.Linear.Y = 0;
            obj.velMsg.Linear.Z = 0;
            obj.velMsg.Angular.X = 0;
            obj.velMsg.Angular.Y = 0;
            obj.velMsg.Angular.Z = w(1);

            % send ROS velocity message
            send(obj.velPub, obj.velMsg)
        end


        % Stop robot - send (zero) velocity commands
        function stop(obj)
            obj.setVelocity( 0, 0)
        end


        % Reads 2D pose
        % returns (x,y) positon [in meters] and (theta) orientation [rad]
        % ptimestamp - pose reading timestamp (in seconds)
        function [x, y, theta, ptimestamp] = readPose(obj)

            % read data from odom topic
            odomMsg = receive(obj.odomSub,3);       % read data w/ 3s timeout
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

            % return pose reading timestamp (ROS time converted in seconds)
            ptimestamp = double(odomMsg.Header.Stamp.Sec) + double(odomMsg.Header.Stamp.Nsec) * 1e-9;

            % hold internal copy of 2D pose + timestap
            obj.poset = [x; y; theta; ptimestamp];
        end



        % Set 2D pose to (x,y,theta)
        % -> note: only works in gazebo
        function setPose(obj, x, y, theta)

            if( obj.isSimulation )

                obj.stop()                      % stop robot
                pause(0.5);                     % force delay

                obj.gzStateMsg.ModelName = obj.gzBotname;           % set model name
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

            else
                fprintf("setting pose is only available in simulation mode\n");
            end
        end


        % Reset turtlebot's pose to origin: (x,y,theta) = (0,0,0)
        % -> note: only works in gazebo
        function resetPose(obj)
            obj.setPose(0.0, 0.0, 0.0);     % set pose to origin (forward oriented to x axis)
        end


        % Get (simulated) Encoders Data
        % returns:
        %   dsr - incremental motion of the right wheel (in meters)
        %   dsl - incremental motion of the left wheel (in meters)
        %   pose2D - [x,y,theta] 2D pose given by the turtlebot's odometry model
        %   timestamp - data reading timestamp (in seconds)
        % note: -> the readEncodersData() and readPose() can't be used together in a loop.
        function [dsr, dsl, pose2D, timestamp] = readEncodersData(obj)

            % kinematic eqs:
            % x1 = x + ds * cos(theta + dtheta/2)
            % y1 = y + ds * sin(theta + dtheta/2)
            % theta1 = theta + dtheta
            %
            % solve for dtheta and ds:
            %: dtheta = theta1 - theta
            %: eq1 + eq2 -> ds = (x1 - x + y1 - y) / ( cos(theta + dtheta/2) + sin(theta + dtheta/2) )

            % load previous pose data
            prev_pose = obj.poset;

            % read current pose
            [x, y, theta, timestamp] = obj.readPose();

            % return pose (given by the internal turtlebot's odometry model)
            pose2D = [x; y; theta];

            % timestamp difference lower than default message timeout
            if( timestamp - prev_pose(4) < 3  )

                % return amount of angular motion
                % dtheta = theta - prev_pose(3);
                dtheta = atan2( sin(theta - prev_pose(3)), cos(theta - prev_pose(3)) );

                % return incremental linear motion
                ds = (x - prev_pose(1) + y - prev_pose(2) ) / ( cos( prev_pose(3) + dtheta/2 ) + sin( prev_pose(3) + dtheta/2) );

            else
                ds = 0;
                dtheta = 0;
            end

            % query wheel baseline (b = 0.16)
            b = obj.wheelbaseline;

            % solve linear system of equations for (dsr and dsl)
            % ds = ( dsr + dsl) / 2
            % dtheta = ( dsr - dsl) / b
            e = inv( [1/2, 1/2; 1/b, -1/b]) * [ds; dtheta];

            % return the solutions
            dsr = e(1);
            dsl = e(2);
        end


        % Get (simulated) Encoders Data with Gaussian Noise
        % receives: noise_std - 2 x 1 vector w/ noise standard deviations of dsr and dsl (in meters)
        % returns:
        %   dsr - incremental motion of the right wheel (in meters)
        %   dsl - incremental motion of the left wheel (in meters)
        %   pose2D - [x,y,theta] 2D pose given by the turtlebot's odometry model
        %   timestamp - data reading timestamp (in seconds)
        % note: -> the readEncodersData() and readPose() can't be used together in a loop.
        function [dsr, dsl, pose2D, timestamp] = readEncodersDataWithNoise(obj, noise_std)

            % check inputs
            if( nargin<2 ) noise_std = zeros(2,1); end

            % read encoders data
            [dsr, dsl, pose2D, timestamp] = obj.readEncodersData();

            % add Gaussian noise to encoder data
            dsr = dsr + randn(1,1) * noise_std(1);
            dsl = dsl + randn(1,1) * noise_std(2);

        end


        % Read lidar data
        % - use rosPlot(scanMsg) to display
        % scanMsg - laser obj struct (ROS message)
        % lddata - lidar data struct
        %       - lddata.Ranges - (radial/polar) distance measurement [meters] (360 x 1) vector
        %       - lddata.Angles - angular measurement [radians] (360 x 1) vector
        %       - lddata.Cartesian - X/Y cartesian data (360 x 2) matrix
        % ldtimestamp - laser scan reading timestamp (in seconds)
        % Note: The lsdata (struct field) arrays could have 'Inf' (or zero) values to represent no laser reflections (representing too near or too far readings). Use getInRangeLidarDataIdx(.) and getOutRangeLidarDataIdx(.) functions to select the desired data.
        function [scanMsg, lddata, ldtimestamp] = readLidar(obj)

            % read data from lidar topic
            scanMsg = receive(obj.lidarSub);

            % return lidar internal data (Ranges, Angles and Cartesian data)
            lddata = rosReadLidarScan(scanMsg);

            % return laser scan reading time (ROS time converted in seconds)
            ldtimestamp = double(scanMsg.Header.Stamp.Sec) + double(scanMsg.Header.Stamp.Nsec) * 1e-9;
        end



        % Get in range lidar data indexs (valid data - obstacles).
        % Returns the linear indexs of several realated arrays: lsdata.Ranges(vidx), or lsdata.Angles(vidx) or lddata.Cartesian(vidx,:).
        function [vidx] = getInRangeLidarDataIdx(obj, lddata)

            %Define lidar min and max laser ranges (see: scanMsg.RangeMin and scanMsg.RangeMax)
            RangeMin = 0.12;
            RangeMax = 3.5;

            % return lsdata.'DATATYPE' indexs of laser redings within range (valid obstacles)
            vidx = find( ~isinf( lddata.Ranges ) & lddata.Ranges >= RangeMin & lddata.Ranges <= RangeMax);
        end



        % Get out of range lidar data indexs (too near or too far readings).
        % Returns the linear indexs of lsdata.Angles(nidx).
        function [nidx] = getOutRangeLidarDataIdx(obj, lddata)

            %Define lidar min and max laser ranges (see: scanMsg.RangeMin and scanMsg.RangeMax)
            RangeMin = 0.12;
            RangeMax = 3.5;

            % return lsdata.'DATATYPE' indexs of out of range readings (too near or too far readings)
            nidx = find( isinf( lddata.Ranges ) | lddata.Ranges < RangeMin  | lddata.Ranges > RangeMax );
        end



        % Read lidar data [previous version, kept for backward compatibility]
        % - use rosPlot(scanMsg) to display
        % scanMsg - laser obj struct (ROS message)
        % xydata - n x 2 data matrix with valid measurements (in cartesian space X-Y)
        % angle - n x 1 data vector (angle value of each measurement)
        function [scanMsg, xydata, angles] = readLidar_(obj)

            % read data from lidar topic
            scanMsg = receive(obj.lidarSub);

            % convert laser scan readings to Cartesian coordinates (and corresponding angles)
            [xydata, angles] = rosReadCartesian(scanMsg);
        end


        % Destructor: clear the workspace of publishers, subscribers, and other ROS-related objects
        % note: stops robot and disconnects ROS, if class object if deleted.
        function delete(obj)
            obj.stop();     % stop robot
            rosshutdown     % shutdown ROS
        end

    end     % end methods

end


