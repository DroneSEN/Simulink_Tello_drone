classdef (Sealed) ryze < matlab.mixin.CustomDisplay & dynamicprops & handle & matlab.mixin.SetGet...
        & ryzeio.internal.RyzeDataUtilityHelper
    %   Connect to a Ryze® Tello drone in MATLAB.
    %
    %   Syntax:
    %       r = ryze
    %       r = ryze(Name/ID)
    %
    %   Description:
    %       r = ryze                Creates connection to a Ryze® Tello drone over Wi-Fi.
    %       r = ryze(Name/ID)       Creates connection to the specified Ryze® Tello drone over Wi-Fi.
    %
    %   Example:
    %   Connect to a Ryze® Tello drone:
    %       r = ryze('Tello');
    %
    %   Connect to a specific Ryze® Tello drone:
    %       r = ryze('TELLO-D31670');
    %
    %   Input Arguments:
    %   Name/ID - Name or unique ID of the Ryze drone
    %
    %   Output Arguments:
    %   r - Ryze drone hardware connection
    %
    %   See also takeoff, land

    %   Copyright 2019-2022 The MathWorks, Inc.

    properties(SetAccess = private)

        % Name of the Ryze drone
        Name

        % Unique ID of the Ryze drone
        ID

        % IP Address of the Ryze drone
        IPAddress

        % Current flying state of the drone. Ryze Tello drone firmware doesn't send
        % this information
        State

        % Boolean property to identify the drone is in station mode or not
        StationMode = false

        % Percentage of battery charge remaining
        BatteryLevel

        % Available cameras on the drone
        AvailableCameras
    end

    properties(Access = private, Hidden)

        % UDP port of the drone
        DronePort

        % Host port to which sensor data packets are sent by the drone
        HostPort

        % Secondary port to which drone sends on demand drone data
        DroneDataHostPort

        % FPV camera address
        FPVCameraAddress

        % Host port to which video packets are sent by the drone
        VideoStreamPort

        % Flag indicating low battery
        LowBattery

        % Current speed of the drone(in m/s)
        Speed

        % Current acceleration of the drone(in m/s^2)
        Accel

        % Current altitude of the drone above takeoff surface(in m)
        Altitude

        % Euler angles representing the rotation from the NED frame (determined at drone startup) to the estimated drone body frame
        Attitude

        % Distance limits for move commands of ryze tello
        DistanceLimits

        % Speed limits for move commands of ryze tello
        SpeedLimits

        % Flag to indicate if the first packet of sensor data has arrived
        FirstPacketReceived

        % Status to indicate if an OK was sent by the drone in response to
        % a control command
        CommandAck

        % Status to indicate if an error was sent by the drone in response
        % to a control command
        CommandError

        % Time out for drone commands
        CommandTimeout

        % Parser output of constructor inputs
        ParserOutput

        % Counter to determine drone disconnection
        NoDataReceivedCount

        % Flag to indicate if MATLAB is still receiving data from drone
        IsTelloConnected = false

        % Timer that keeps reading sensor data packet from the drone
        SensorTimerObj

        % Timer that keeps reading drone status data packet from the drone
        DroneDataTimerObj

        % A key for the ConnectionMap
        ConnectionKey

        % Drone ID returned by the drone
        DetectedID

        % Default IP address of all Tello drones
        DefaultIPAddress = ryzeio.internal.RyzeConstants.TelloDefaultIP;

        % Port to send commands and receive response from the drone
        CommandHostPort

        % Timer that keeps reading the drone ACK to various Commands and
        % the battery status queried by pingserver
        CommandResponseTimerObj

    end

    properties(SetAccess = private, Hidden)

        % Connection to send commands to drone and receive ACK
        CommandConnection

        % UDP connection handle to receive on-demand drone status data like
        % the SSID
        SecondaryDataConnection
    end

    properties(Access = private, Constant, Hidden)

        % Store the Asyncio connection which reads sensor data stream from
        % all connected drones
        SensorConnection = containers.Map();

        % Maintain a Map of existing Ryze Tello drone connections
        ConnectionMap = containers.Map();

        % Packet to be sent to drone in non-SDK mode to retrieve SSID
        SSIDPacket = uint8([204 88 0 124 72 17 0 1 0 27 136]);


    end
    methods
        %% get and set methods
        function battery =  get.BatteryLevel(obj)
            % Get function for BatteryLevel to prevent internal process stalling while reading this property in a loop

            battery = obj.BatteryLevel;
            drawnow;
        end

        function state =  get.State(obj)
            % Get function for State property to convert enum to char

            state = lower(string(obj.State));
            drawnow;
        end

        function set.CommandTimeout(obj,timeout)
            % Set command timeout if the timeout duration is more than the
            % default command timeout
            if timeout >= ryzeio.internal.RyzeConstants.DefaultCommandTimeout
                obj.CommandTimeout = timeout;
            end
        end
    end

    methods(Access = public)
        %% Constructor
        function obj = ryze(varargin)

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj,varargin{:}));

                narginchk(0,1);

                % Register message catalog
                ryzeio.internal.Utility.loadResources();

                % Validate the MATLAB preference for H.264 decoding
                % This function will create a new MATLAB_H264 preference if one doesn't exist
                initH264pref(obj);

                % Property initializations
                initProperties(obj, varargin{:});

                % Before establishing connection ensure its not a duplicate
                % connection
                if checkConnectionExists(obj)
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionExists');
                end

                % Perform handshake and establish UDP connection with drone
                connectDrone(obj);

                % Setup drone
                setupDrone(obj);

                % Save the drone details
                saveDroneProperties(obj);
            catch e

                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end
    end

    methods
        %% Destructor
        function delete(obj)

            % Shut down the drone motors first to ensure that the drone
            % lands when the user clears the drone connection during flight
            if obj.IsTelloConnected
                try
                    land(obj);
                catch
                end
            end

            % Stop and clear all timer objects
            stopDroneTimers(obj);
            % Close all Asyncio connections to the drone
            closeDroneConnections(obj);

            % Remove the drone entry from the Map of existing connections.
            % The ConnectionKey is initialized and an entry is added to the
            % Map only after the drone connection is active. Hence remove
            % the entry only if the connection is active i.e the
            % ConnectionKey is part of connection map
            if isKey(obj.ConnectionMap,obj.ConnectionKey)

                % Remove the drone entry from active drone connections
                ryzeio.internal.Utility.updateActiveConnections(obj,'remove');

                % Close the drone sensor data stream connection when the
                % last drone object is being deleted
                if obj.ConnectionMap.Count == 1 && obj.SensorConnection.Count ~= 0
                    stopSensorDataCollection(obj.SensorConnection('SensorConnectionObj'));
                    % Remove the sensor connection object handle
                    remove(obj.SensorConnection,'SensorConnectionObj');
                end

                % Remove the entry from the connected drones Map
                remove(obj.ConnectionMap,obj.ConnectionKey);
            elseif obj.ConnectionMap.Count == 0 && obj.SensorConnection.Count ~= 0
                % Close the drone sensor data stream connection if the
                % drone object creation fails.
                stopSensorDataCollection(obj.SensorConnection('SensorConnectionObj'));
                % Remove the sensor connection object handle
                remove(obj.SensorConnection,'SensorConnectionObj');
            end
        end
    end

    methods(Access = public)
        %% Drone control functions
        function takeoff(obj, varargin)
            %   Drone takeoff function
            %
            %   Syntax:
            %       takeoff(r)
            %       takeoff(r, 'WaitUntilDone', false)
            %
            %   Description:
            %       Takes off the Ryze drone from the ground.
            %
            %   Example:
            %       r = ryze('tello');
            %       takeoff(r);
            %       takeoff(r, 'WaitUntilDone', false);
            %
            %   Input Arguments:
            %   r    - ryze tello object
            %   Name-Value pairs:
            %   WaitUntilDone - Name Value pair to control blocking or non-blocking mode of takeoff (default = true)
            %
            %   See also land, abort
        
            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj));
        
                narginchk(1, 3);
        
                % Parse optional parameters
                p = inputParser;
                addParameter(p, 'WaitUntilDone', true, @islogical);
                parse(p, varargin{:});
                blocking = p.Results.WaitUntilDone;
        
                % Check whether drone connection is still active
                checkDroneConnection(obj);
        
                % Display takeoff command failed to execute due to low battery
                if(obj.LowBattery)
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:insufficientCharge', 'takeoff');
                end
        
                % Send takeoff command
                obj.CommandAck = false;
                write(obj.CommandConnection, uint8('takeoff'));
        
                % Block MATLAB till ACK is received from the drone if blocking mode is enabled
                if(blocking)
                    try
                        waitForCommandAck(obj);
                    catch e
                        throwAsCaller(e);
                    end
                    obj.State = ryzeio.internal.DroneStateEnum.Hovering;
                else
                    % Wait to see if there are any errors sent by the drone
                    t = tic;
                    while(toc(t)<0.25)
                        if(obj.CommandError)
                            ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:commandError');
                        end
                        pause(0.05);
                    end
                end
            catch e
                integrateErrorKey(obj, e.identifier);
                throwAsCaller(e);
            end
        end


        function rc(obj,vx, vy, vz, d)
            %   Drone RC function
            %
            %   Syntax:
            %       rc(r)
            %       rc(r, 'WaitUntilDone', false)
            %
            %   Description:
            %       Send RC command to the drone with speed as parameters.
            %       Speeds are expressed in cm/s
            %
            %   Example:
            %
            %   Input Arguments:
            %
            try 
                command=char(sprintf("rc %d %d %d %d",int16(vx),int16(vy),int16(vz),int16(d)));
                write(obj.CommandConnection, uint8(command));
            catch e 
                throwAsCaller(e);
            end
        end 

        function activateMissionPad(obj)
            % Function to activate mission pad control on the Tello drone
            % This command "mon" will enable mission pad detection.
            try
                obj.CommandAck = false;
                obj.CommandError = false;
                write(obj.CommandConnection, uint8('mon'));
                t = tic;
                while (toc(t) < obj.CommandTimeout && ~obj.CommandAck)
                    if (obj.CommandError)
                        return;
                    end
                    drawnow;
                end
            catch e
                throwAsCaller(e);
            end
        end

        function deactivateMissionPad(obj)
            % Function to deactivate mission pad control on the Tello drone
            % This command "moff" will disable mission pad detection.
            try
                obj.CommandAck = false;
                obj.CommandError = false;
                write(obj.CommandConnection, uint8('moff'));
                t = tic;
                while (toc(t) < obj.CommandTimeout && ~obj.CommandAck)
                    if (obj.CommandError)
                        return;
                    end
                    drawnow;
                end
            catch e
                throwAsCaller(e);
            end
        end
        
        function switch_camera(obj,orientation)
            %   Drone switch camera function
            %
            %   Syntax:
            %       switch_camera(orientation)
            %
            %   Description:
            %       Send a command to switch the direction of the camera
            %
            %   Example:
            %
            %   Input Arguments:
            %      Direction:
            %       0 : Forward
            %       1 : Downard
            %       2 : Both directions
            %

            try 
                if (0 <= orientation) && (orientation <= 2)
                    command = char(sprintf("downvision %d", uint16(orientation)));
                    write(obj.CommandConnection,uint8(command));
                end
            catch e 
                throwAsCaller(e);
            end 
        end         


        function land(obj, varargin)
            %   Drone land function
            %
            %   Syntax:
            %       land(r)
            %       land(r, 'WaitUntilDone', false)
            %
            %   Description:
            %       Initiates smooth land of the drone.
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       land(r);
            %       land(r, 'WaitUntilDone', false);
            %
            %   Input Arguments:
            %   r    - ryze tello object
            %   Name-Value pairs:
            %   WaitUntilDone - Name Value pair to control blocking or non-blocking mode of land (default = true)
            %
            %   See also takeoff, abort
        
            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj));
        
                narginchk(1, 3);
        
                % Parse optional parameters
                p = inputParser;
                addParameter(p, 'WaitUntilDone', true, @islogical);
                parse(p, varargin{:});
                blocking = p.Results.WaitUntilDone;
        
                % Check whether drone connection is still active
                checkDroneConnection(obj);
        
                % Send land command
                obj.CommandAck = false;
                write(obj.CommandConnection, uint8('land'));
        
                % Block MATLAB till ACK is received from the drone if blocking mode is enabled
                if(blocking)
                    try
                        waitForCommandAck(obj);
                    catch e
                        throwAsCaller(e);
                    end
                    obj.State = ryzeio.internal.DroneStateEnum.Landed;
                else
                    % Wait to see if there are any errors sent by the drone
                    t = tic;
                    while(toc(t) < 0.25)
                        if(obj.CommandError)
                            ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:commandError');
                        end
                        pause(0.05);
                    end
                end
            catch e
                integrateErrorKey(obj, e.identifier);
                throwAsCaller(e);
            end
        end

        function abort(obj)
            %   Function to abort drone flight
            %
            %   Syntax:
            %       abort(r)
            %
            %   Description:
            %       Shuts down drone motors instantaneously. Execute takeoff command
            %       to start flying the drone again.
            %
            %       This command works only with a Tello EDU and errors out if executed on a standard
            %       Tello.
            %
            %   Example:
            %       r = ryze("TelloEDU");
            %       takeoff(r);
            %       abort(r);
            %
            %   Input Arguments:
            %   r    - ryze tello object
            %
            %   See also takeoff, land

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj));

                % Error out if abort function is tried on a standard Tello
                if(~strcmpi(obj.Name, ryzeio.internal.TelloTypeEnum.TelloEDU))
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:unsupportedFunction', 'abort');
                end
                narginchk(1,1);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Send abort command
                write(obj.CommandConnection, uint8('emergency'));

                % This is a workaround for g2774564. Drone doesn't return
                % any reply for 'emergency' command packet. Hence don't
                % check for the response. If manufacturer resolves the
                % issue as part of g2809627, we can revert this change
                obj.State = ryzeio.internal.DroneStateEnum.Landed;
            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        %% Navigation data read methods

        function [speed, time] = readSpeed(obj)
            %   Function to read current speed(in m/s) of the drone
            %
            %   Syntax:
            %       [speed, time] = readSpeed(r);
            %
            %   Description:
            %       Function to read current speed of the drone in m/s,
            %       with respect to the North-East-Down frame, estimated at
            %       drone startup.
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       [speed, time] = readSpeed(r);
            %
            %   Input Arguments:
            %   r       - ryze tello object
            %
            %   Output Arguments:
            %   speed   - 1*3 double vector of drone speed(in m/s) along X, Y, and Z axes
            %   time    - timestamp in date-time format
            %
            %   See also readHeight, readOrientation, land

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj));

                narginchk(1,1);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Tello drone returns the speed in dm/s
                speed   = obj.Speed.Value/10;
                time = obj.Speed.Time;
                drawnow;
            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        %% TODO : Add to github
        function [accel, time] = readAccel(obj)
            %   Function to read current acceleration(in m/s^2) of the drone
            %
            %   Syntax:
            %       [accel, time] = readAccel(r);
            %
            %   Description:
            %       Function to read current acceleration of the drone in
            %       m/s^2, with respect to the North-East-Down frame, 
            %       estimated at drone startup.
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       [accel, time] = readAccel(r);
            %
            %   Input Arguments:
            %   r       - ryze tello object
            %
            %   Output Arguments:
            %   acceleration - 1*3 double vector of drone acceleration
            %                  (in m/s^2) along X, Y, and Z axes
            %   time    - timestamp in date-time format
            %
            %   See also readHeight, readOrientation, land

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj));

                narginchk(1,1);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Tello drone returns the speed in cm/s^2
                accel   = obj.Accel.Value;
                time = obj.Accel.Time;
                drawnow;
            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        function battery = readBattery(obj)
            %   Function to read current battery level of the drone
            %
            %   Syntax:
            %       [battery, time] = readBattery(r);
            %
            %   Description:
            %       Function to read current battery of the drone
            %   Example:
            %       r = ryze();
            %       readBattery(r);
            %
            %   Input Arguments:
            %   r       - ryze tello object
            %
            %   Output Arguments:
            %   battery level - Battery level of the drone
            %   time    - timestamp in date-time format
            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj));

                narginchk(1,1);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Tello drone returns the battery
                battery   = obj.BatteryLevel;
                drawnow;
            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end

        end

        %% END TODO

        function [height, time] = readHeight(obj)
            %   Function returns the height(in m) of the drone relative to the
            %   takeoff surface
            %
            %   Syntax:
            %       [height, time] = readHeight(r);
            %
            %   Description:
            %       Function to read current height of the drone in m
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       [height, time] = readHeight(r);
            %
            %   Input Arguments:
            %   r       - ryze tello object
            %
            %   Output Arguments:
            %   height  - height of the drone from takeoff surface (in m)
            %   time    - timestamp in date-time format
            %
            %   See also readSpeed, readOrientation, land

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj));

                narginchk(1,1);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Drone returns the height in cm
                height  = obj.Altitude.Value/100;
                time = obj.Altitude.Time;
                drawnow
            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        function [angles, time] = readOrientation(obj)
            %   Function to read current orientation of the drone with respect to the
            %   NED frame calculated at drone startup.
            %
            %   Syntax:
            %       [angles, time] = readOrientation(r);
            %
            %   Description:
            %       Function to read current orientation of the drone with respect to the
            %       NED frame calculated at drone startup. 0 radians along Z-axis being
            %       the direction the drone faces during startup.
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       [angles, time] = readOrientation(r);
            %
            %   Input Arguments:
            %   r       - ryze object
            %
            %   Output Arguments:
            %   angles  - 1*3 vector of Euler rotation angles along ZYX axes (in radians)
            %   time    - timestamp in date-time format
            %
            %   See also readSpeed, readHeight, land

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj));

                narginchk(1,1);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                angles  = deg2rad(obj.Attitude.Value);
                time = obj.Attitude.Time;
                drawnow
            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        %% Navigation functions

        function moveforward(obj, varargin)
            %   Function to move the drone forward by the specified
            %   duration (in s) or distance(in m)
            %
            %   Syntax:
            %       moveforward(r)
            %       moveforward(r, duration)
            %       moveforward(r, duration, Name, Value)
            %       moveforward(r, Name, Value)
            %
            %   Description:
            %       Function to move the drone forward by the specified
            %       duration (in s) or distance(in m).
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       moveforward(r,'Distance',3,'Speed',0.5);
            %
            %   Input Arguments:
            %   r           - ryze object
            %   duration    - Duration(in s) for which the drone has to move
            %
            %   Name-Value pairs:
            %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
            %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
            %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
            %
            %   See also moveback, land, abort, turn

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj,varargin{:}));

                narginchk(1, 7);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'forward', varargin);
            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        function moveback(obj, varargin)
            %   Function to move the drone back by the specified
            %   duration (in s) or distance(in m)
            %
            %   Syntax:
            %       moveback(r)
            %       moveback(r, duration)
            %       moveback(r, duration, Name, Value)
            %       moveback(r, Name, Value)
            %
            %   Description:
            %       Function to move the drone back by the specified
            %       duration (in s) or distance(in m).
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       moveback(r,'Distance',3,'Speed',0.5);
            %
            %   Input Arguments:
            %   r           - ryze object
            %   duration    - Duration(in s) for which the drone has to move
            %
            %   Name-Value pairs:
            %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
            %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
            %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
            %
            %   See also moveright, land, abort, turn

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj,varargin{:}));

                narginchk(1, 7);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'back', varargin);

            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        function moveright(obj, varargin)
            %   Function to move the drone right by the specified
            %   duration (in s) or distance(in m)
            %
            %   Syntax:
            %       moveright(r)
            %       moveright(r, duration)
            %       moveright(r, duration, Name, Value)
            %       moveright(r, Name, Value)
            %
            %   Description:
            %       Function to move the drone right by the specified
            %       duration (in s) or distance(in m).
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       moveright(r,'Distance',3,'Speed',0.5);
            %
            %   Input Arguments:
            %   r           - ryze object
            %   duration    - Duration(in s) for which the drone has to move
            %
            %   Name-Value pairs:
            %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
            %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
            %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
            %
            %   See also moveleft, land, abort, turn

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj,varargin{:}));

                narginchk(1, 7);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'right', varargin);

            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        function moveleft(obj, varargin)
            %   Function to move the drone left by the specified
            %   duration (in s) or distance(in m)
            %
            %   Syntax:
            %       moveleft(r)
            %       moveleft(r, duration)
            %       moveleft(r, duration, Name, Value)
            %       moveleft(r, Name, Value)
            %
            %   Description:
            %       Function to move the drone forward by the specified
            %       duration (in s) or distance(in m).
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       moveleft(r,'Distance',3,'Speed',0.5);
            %
            %   Input Arguments:
            %   r           - ryze object
            %   duration    - Duration(in s) for which the drone has to move
            %
            %   Name-Value pairs:
            %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
            %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
            %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
            %
            %   See also moveup, land, abort, turn

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj,varargin{:}));

                narginchk(1, 7);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'left', varargin);

            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        function moveup(obj, varargin)
            %   Function to move the drone up by the specified
            %   duration (in s) or distance(in m)
            %
            %   Syntax:
            %       moveup(r)
            %       moveup(r, duration)
            %       moveup(r, duration, Name, Value)
            %       moveup(r, Name, Value)
            %
            %   Description:
            %       Function to move the drone up by the specified
            %       duration (in s) or distance(in m).
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       moveup(r,'Distance',3,'Speed',0.5);
            %
            %   Input Arguments:
            %   r           - ryze object
            %   duration    - Duration(in s) for which the drone has to move
            %
            %   Name-Value pairs:
            %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
            %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
            %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
            %
            %   See also movedown, land, abort, turn

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj,varargin{:}));

                narginchk(1, 7);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'up', varargin);

            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        function movedown(obj, varargin)
            %   Function to move the drone down by the specified
            %   duration (in s) or distance(in m)
            %
            %   Syntax:
            %       movedown(r)
            %       movedown(r, duration)
            %       movedown(r, duration, Name, Value)
            %       movedown(r, Name, Value)
            %
            %   Description:
            %       Function to move the drone down by the specified
            %       duration (in s) or distance(in m).
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       movedown(r,'Distance',3,'Speed',0.5);
            %
            %   Input Arguments:
            %   r           - ryze object
            %   duration    - Duration(in s) for which the drone has to move
            %
            %   Name-Value pairs:
            %   Distance    - Name value pair to specify relative distance(in m). Range: 0.2m - 5m (default = 0.2m)
            %   Speed       - Name Value pair to specify the speed(in m/s) of move. Range: 0.1m/s - 1m/s (default = 0.4m/s)
            %   WaitUntilDone - Name Value pair to specify if the move is blocking or non-blocking (default = true)
            %
            %   See also moveup, land, abort, turn

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj,varargin{:}));

                narginchk(1, 7);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Execute move API to move the drone forward by a relative distance with the
                % specified speed
                basicMoveImpl(obj, 'down', varargin);

            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        function move(obj, varargin)
            %   Function to move the drone along 3 axis at the same time
            %
            %   Syntax:
            %       move(r, relativeDistance)
            %       move(r, relativeDistance, Name, Value)
            %
            %   Description:
            %       Function to move the drone by the relative coordinate
            %       position
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       move(r, [1 1 -1], 'Speed', 0.5);
            %
            %   Input Arguments:
            %   r                 - ryze object
            %   relativeDistance  - Relative coordinate value in [X,Y,Z]
            %
            %   Name-Value pairs:
            %   Speed             - Name value pair to control speed of move (default = 0.4m/s). Range: 0.1m/s - 1m/s
            %   WaitUntilDone     - Name Value pair to control blocking or non-blocking mode of move (default = true)
            %
            %   See also takeoff, land, abort, turn

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj,varargin{:}));

                narginchk(2,6);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Validate if the input parameters are within range
                [coordinate, speed, blocking] = ryzeio.internal.Utility.validateMoveParams(varargin, obj.DistanceLimits, obj.SpeedLimits);

                % Convert units to cm
                coordinate = coordinate.*100;
                speed = speed * 100;

                % Set speed and then send move command
                obj.CommandAck = false;
                command = char(sprintf("speed %d", round(speed)));
                write(obj.CommandConnection, uint8(command));
                waitForCommandAck(obj);

                obj.CommandAck = false;
                obj.CommandError = false;
                if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Hovering)))
                    obj.State = ryzeio.internal.DroneStateEnum.Flying;
                end

                % Convert coordinates to NED convention and send move SDK
                % command
                obj.CommandAck = false;
                write(obj.CommandConnection, uint8(char(sprintf("go %d %d %d %d",round(coordinate(1)), round(-1*coordinate(2)), round(-1*coordinate(3)), round(speed)))));

                % Block MATLAB till ACK is received from the drone
                if(blocking)
                    try
                        waitForCommandAck(obj);
                    catch e
                        if(strcmpi(e.identifier, "MATLAB:ryze:general:commandError"))
                            if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Flying)))
                                obj.State = ryzeio.internal.DroneStateEnum.Hovering;
                            end
                        end
                        throwAsCaller(e);
                    end
                else
                    % Wait to see if there are any errors sent by the drone
                    t = tic;
                    while(toc(t)<0.25)
                        if(obj.CommandError)
                            if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Flying)))
                                obj.State = ryzeio.internal.DroneStateEnum.Hovering;
                            end
                            ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:commandError');
                        end
                        pause(0.05);
                    end
                end
            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        function turn(obj, angle, varargin)
            %   Function to turn the drone by a relative angle in radians
            %
            %   Syntax:
            %       turn(r, angle)
            %       turn(r, angle, 'WaitUntilDone', false)
            %
            %   Description:
            %       Turns the drone by the specified angle in radians
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       turn(r, deg2rad(-45));
            %       turn(r, deg2rad(-45), 'WaitUntilDone', false);
            %
            %   Input Arguments:
            %   r       - ryze object
            %   angle   - angle in radians
            %   Name-Value pairs:
            %   WaitUntilDone - Name Value pair to control blocking or non-blocking mode of turn (default = true)
            %
            %   See also takeoff, land, abort
        
            try
                narginchk(2, 4);
        
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj, angle));
        
                % Check whether drone connection is still active
                checkDroneConnection(obj);
        
                % Validate the angle and convert it to degrees
                angle = rad2deg(angle);
                ryzeio.internal.Utility.validateAngle(angle, [-360 360]);
        
                % Parse optional parameters
                p = inputParser;
                addParameter(p, 'WaitUntilDone', true, @islogical);
                parse(p, varargin{:});
                blocking = p.Results.WaitUntilDone;
        
                obj.CommandAck = false;
                if(angle >= 0)
                    write(obj.CommandConnection, uint8(sprintf('cw %d', round(angle))));
                else
                    write(obj.CommandConnection, uint8(sprintf('ccw %d', round(abs(angle)))));
                end
        
                % Block MATLAB till ACK is received from the drone if blocking mode is enabled
                if(blocking)
                    try
                        waitForCommandAck(obj);
                    catch e
                        throwAsCaller(e);
                    end
                else
                    % Wait to see if there are any errors sent by the drone
                    t = tic;
                    while(toc(t)<0.25)
                        if(obj.CommandError)
                            ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:commandError');
                        end
                        pause(0.05);
                    end
                end
            catch e
                integrateErrorKey(obj, e.identifier);
                throwAsCaller(e);
            end
        end


        function flip(obj, direction)
            %   Drone flip function
            %
            %   Syntax:
            %       flip(r, direction)
            %
            %   Description:
            %       Flips the drone in the desired direction
            %
            %   Example:
            %       r = ryze();
            %       takeoff(r);
            %       flip(r, 'forward');
            %
            %   Input Arguments:
            %       r - ryze tello object
            %       direction - Direction to flip. Possible values are:
            %       ["forward", "back", "left", "right"]
            %
            %   See also takeoff, land, abort

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj));

                narginchk(2,2);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Validate the flip direction
                direction = ryzeio.internal.Utility.validateFlipDirection(direction);

                obj.CommandAck = false;
                if(strcmpi(direction, 'forward'))
                    write(obj.CommandConnection, uint8('flip f'));
                elseif(strcmpi(direction, 'back'))
                    write(obj.CommandConnection, uint8('flip b'));
                elseif(strcmpi(direction, 'left'))
                    write(obj.CommandConnection, uint8('flip l'));
                elseif(strcmpi(direction, 'right'))
                    write(obj.CommandConnection, uint8('flip r'));
                end
                try
                    waitForCommandAck(obj);
                catch e
                    if(strcmpi(e.identifier, "MATLAB:ryze:general:commandError"))
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:flipError');
                    end
                end
            catch e
                integrateErrorKey(obj, e.identifier);

                throwAsCaller(e);
            end
        end

        function cameraObj = camera(obj, varargin)
            %   This function creates a camera object to access the drone's FPV camera.
            %
            %   camObj = camera(ryzeObj, name) returns a camera object, camObj, that can be used to
            %   access the specified drone camera.
            %
            %   Usage:
            %       Construct a camera object to connect to a Ryze drone camera
            %
            %       camObj = camera(ryzeObj, 'FPV');
            %       OR
            %       camObj = camera(ryzeObj);
            %
            %       % Preview a stream of image frames.
            %       preview(camObj);
            %
            %       % Acquire and display a single image frame.
            %       img = snapshot(camObj);
            %       imshow(img);

            try
                % Initialize clean up function
                c = onCleanup(@() integrateData(obj, varargin{:}));

                narginchk(1,2);

                % Check whether drone connection is still active
                checkDroneConnection(obj);

                % Drone camera is unavailable when the drone is in station
                % mode
                if obj.StationMode
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:cameraUnavailable',obj.ID);
                end

                % Select the drone's FPV camera by default
                if(1 == nargin)
                    droneCamera = "FPV";
                else
                    % Second input is the drone camera
                    droneCamera = varargin{1};
                end

                % String to char conversion
                droneCamera = char(droneCamera);

                % Validate if the droneCamera inputted is one of the available drone
                % cameras
                try
                    droneCamera = validatestring(droneCamera, obj.AvailableCameras);
                catch
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:invalidCamera');
                end

                % Throw error if H264 decoding using openh264 library is disabled
                if(~getpref('MATLAB_H264', 'Status'))
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:h264Disabled');
                end

                % Start video stream in Tello drone
                obj.CommandAck = false;
                write(obj.CommandConnection, uint8('command'));
                try
                    waitForCommandAck(obj);
                catch
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end

                obj.CommandAck = false;
                write(obj.CommandConnection, uint8('streamon'));
                try
                    waitForCommandAck(obj);
                catch
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end

                % Create camera addon connection
                cameraObj = ryzeioaddons.Camera(obj.CommandConnection, string(droneCamera), obj.FPVCameraAddress, obj.VideoStreamPort);
            catch e
                integrateErrorKey(obj, e.identifier);

                % Stop video stream in case of an error
                write(obj.CommandConnection, uint8('streamoff'));
                throwAsCaller(e);
            end
        end
    end

    %% Internal functions
    methods(Access = private)

        function output = parseinputs(~, inputs)
            % Parse the inputs entered by the user

            nInputs = numel(inputs);
            output.Name = [];

            % If the user enters no input argument
            if nInputs == 0
                return;
            end

            % Validate the drone name, ID or IP address entered by the user
            if nInputs > 0
                if (contains(inputs{1},lettersPattern) || isempty(inputs{1}) || inputs{1} == "")
                    % If user input contains alphabets, it would be either
                    % be Name or ID. Validate the user provided drone
                    % name/ID
                    output = ryzeio.internal.Utility.validateDroneName(inputs{1});
                else
                    % If user input doesn't contain any alphabets it would
                    % be the IP address. Validate the user provided drone
                    % IP Address
                    output.IPAddress = ryzeio.internal.Utility.validateDroneIPAddress(inputs{1});
                end
            end
        end

        function initProperties(obj, varargin)
            % Function to initialize class properties

            try
                % Parse the input arguments
                obj.ParserOutput = parseinputs(obj, varargin);

                % Initialize drone Name
                if(isempty(obj.ParserOutput.Name))
                    obj.Name   = string(ryzeio.internal.TelloTypeEnum.Tello);
                else
                    % Drone name provided by the user
                    obj.Name = string(obj.ParserOutput.Name);
                end

                % Initialize drone ID
                if isfield(obj.ParserOutput,"ID")
                    % Drone ID provided by the user
                    obj.ID = string(obj.ParserOutput.ID);
                end

                % Initialize drone IP address
                if isfield(obj.ParserOutput,"IPAddress")
                    % Drone IP address provided by the user
                    obj.IPAddress = string(obj.ParserOutput.IPAddress);
                else
                    % Find the IP address of the drone from the user
                    % provided drone ID
                    obj.IPAddress = ryzeio.internal.Utility.findIPFromID(obj.ID);

                    % If the drone IP address is not found, connect to the
                    % default drone IP address
                    if obj.IPAddress == ""

                        % If the Mock Tello drone is being used, assign the Mock
                        % Tello default IP address as Tello drone's default IP
                        % address
                        [mockEnabled, mockDefaultIPAddress] = ryzeio.internal.Utility.validateTestEnvironment;

                        if mockEnabled
                            obj.DefaultIPAddress = mockDefaultIPAddress;
                        end

                        % Assign the default Tello IP address
                        obj.IPAddress = string(obj.DefaultIPAddress);
                    end
                end

                % Initialize State to "Unknown" since this data is not sent by
                % Tello firmware
                obj.State = ryzeio.internal.DroneStateEnum.Landed;

                % Initialize UDP port to send command packets to the ryze drone
                obj.DronePort   = 8889;
                obj.HostPort    = 8890;
                obj.DroneDataHostPort = 8899;
                obj.FPVCameraAddress = "";
                obj.VideoStreamPort = 11111;
                obj.DetectedID = '';

                % Assign initial values for drone settings/properties
                obj.CommandTimeout = ryzeio.internal.RyzeConstants.DefaultCommandTimeout;
                obj.BatteryLevel = 100;
                obj.FirstPacketReceived = false;
                obj.LowBattery = false;

                % Assign initial values for navigation data
                obj.Speed = struct("Value", [], "Time", []);
                obj.Accel = struct("Value", [], "Time", []);
                obj.Altitude = struct("Value", [], "Time", []);
                obj.Attitude = struct("Value", [], "Time", []);
                obj.DistanceLimits = [0.2 5];
                obj.SpeedLimits = [0.1 1];

                % The ryze Tello drone has only a forward facing camera that can
                % be accessed from MATLAB
                obj.AvailableCameras = ["FPV"];

                % Assign property values for drones in station mode
                initStationMode(obj);

                % Find the host port to be used for sending commands and
                % receiving ACK
                % TODO: Add configuration to select a Command host port
                % to use drone
                % Idea : Empty == Search an available port
                %        Otherwise, select the given port
                obj.CommandHostPort = ryzeio.internal.Utility.findPort(obj.IPAddress);
                % obj.CommandHostPort = 8889;
            catch e
                throwAsCaller(e);
            end

        end

        function initH264pref(~)
            % Function to check if H.264 support is enabled/disabled
            % Create a new preference, if one does not exist

            if(~ispref('MATLAB_H264', 'Status'))
                addpref('MATLAB_H264', 'Status', true);
            end
        end

        function connectDrone(obj)
            % Function to perform handshake and establish UDP connection with the drone

            try
                % Start reading the sensor data streamed by the drone
                initDroneSensorConnection(obj);

                % Establish UDP connection to send command and receive
                % command ACK
                obj.CommandConnection = ryzeio.internal.WifiConnection(obj.IPAddress, obj.DronePort, obj.CommandHostPort);
                % Initialize and start timer to read data from the AsyncIO
                % buffer
                obj.CommandResponseTimerObj = internal.IntervalTimer(0.5);
                addlistener(obj.CommandResponseTimerObj, 'Executing', @(~, ~)obj.readCommandResponseData);
                obj.CommandResponseTimerObj.start();

                % Establish UDP connection to get SSID data(DroneData) and
                obj.SecondaryDataConnection = ryzeio.internal.WifiConnection(obj.IPAddress, obj.DronePort, obj.DroneDataHostPort);
                % Initialize and start timer to read data from the AsyncIO
                % buffer for drone's status data like SSID
                obj.DroneDataTimerObj = internal.IntervalTimer(0.5);
                addlistener(obj.DroneDataTimerObj, 'Executing', @(~, ~)obj.readDroneData);
                obj.DroneDataTimerObj.start();
            catch e
                throwAsCaller(e);
            end
        end

        function setupDrone(obj)
            % Function to setup the following after connection:
            %   1. Set the Tello drone in command mode
            %   2. Start a thread to read the battery state every 1s
            %   3. Read the SSID of the drone
            %   4. Setup connection to start reading the drone sensor stream data

            try
                % Set the ryze Tello drone in command mode
                obj.CommandAck = false;
                write(obj.CommandConnection, uint8('command'));

                % Wait for the ACK from the previous command
                try
                    waitForCommandAck(obj);
                catch
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end

                % Wait till the first sensor data packet is received to
                % ensure the object display has the latest value for
                % BatteryLevel
                t = tic;
                while(~obj.FirstPacketReceived && toc(t) < obj.CommandTimeout)
                    drawnow
                end

                % Get the SSID of the drone
                getDroneSSID(obj);

                if(isTelloEDU(obj))
                    obj.Name = string(ryzeio.internal.TelloTypeEnum.TelloEDU);
                else
                    % Error out if user entered TelloEDU and the drone that is connected is Tello
                    if(strcmpi(obj.Name, ryzeio.internal.TelloTypeEnum.TelloEDU))
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:incorrectDroneName', string(obj.Name));
                    end
                    obj.Name = string(ryzeio.internal.TelloTypeEnum.Tello);
                end

                % Update drone connection status, if connection is successful
                obj.IsTelloConnected = true;
            catch e
                throwAsCaller(e);
            end
        end

        function getDroneSSID(obj)
            % Function to fetch the SSID of the drone using secondary data
            % connection

            try
                % Send GET command to read drone's SSID and read the
                % response packet.
                write(obj.SecondaryDataConnection, obj.SSIDPacket);
                t = tic;
                while(toc(t) < obj.CommandTimeout && isempty(obj.DetectedID))
                    % Wait till the SSID of the drone is received and
                    % assigned
                    drawnow
                end

                % Validate drone ID and assign drone name
                obj.ID = ryzeio.internal.Utility.validateDroneID(obj.ID, obj.DetectedID);

                % To make the drone switch from raw packet to SDK mode
                obj.CommandAck = false;
                write(obj.CommandConnection, uint8('command'));
                % Wait for the ACK from the previous command
                try
                    waitForCommandAck(obj);
                catch e
                    if strcmpi(e.identifier, 'MATLAB:ryze:general:commandError')
                        % g2240014. This use case came from tello mock
                        % where we don't switch to raw packet mode. Hence,
                        % the 2ND command packet is replied with 'ok'.
                        % But in case of an actual tello,when the 2ND
                        % 'command' packet is sent to switch the drone from
                        % raw packet to SDK mode, drone replies back with
                        % 'unknown command:command?' error. Hence ignoring
                        % the error.
                    else
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                    end
                end


                % We don't need this UDP connection anymore (This was used to only get the drone ID)
                close(obj.SecondaryDataConnection);
                % Stop timer and clear timer object
                if(~isempty(obj.DroneDataTimerObj) && isvalid(obj.DroneDataTimerObj))
                    obj.DroneDataTimerObj.stop();
                    obj.DroneDataTimerObj = [];
                end
            catch e
                throwAsCaller(e);
            end
        end

        function enableStationModeImpl(obj,ssid,password)
            % Function to set the Tello EDU drone in station mode
            try
                if(strcmpi(obj.Name, ryzeio.internal.TelloTypeEnum.Tello))
                    % Only Tello EDU drones support station mode. Error out
                    % for normal Tello drones
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:unsupportedStationMode', string(obj.Name));

                else

                    obj.CommandAck = false;
                    command = char(sprintf("ap %s %s",ssid,password));
                    write(obj.CommandConnection, uint8(command));
                    try
                        waitForCommandAck(obj);
                        obj.StationMode = true;
                    catch e
                        if(strcmpi(e.identifier, "MATLAB:ryze:general:commandError"))
                            ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:stationModeError', string(obj.Name));
                        end
                    end
                end
            catch e
                obj.StationMode = false;
                throwAsCaller(e);
            end
        end

        function initStationMode(obj)
            % Function to assign the property values for drones in station
            % mode
            % if ~any(strcmpi(obj.IPAddress,obj.DefaultIPAddress))
            %     % A Tello EDU drone in station mode won't have the default
            %     % IP address. Its assigned a dynamic IP address by the
            %     % access point
            %     obj.StationMode = true;
            % 
            %     % Drone camera can't be accessed in station mode
            %     obj.AvailableCameras = [];
            % end
        end

        function check= checkConnectionExists(obj)
            % Function to check whether user is trying to connect to a
            % already connected drone or connection to a drone in station mode
            % already exists

            if ~isempty(obj.ID) && any(startsWith(obj.ConnectionMap.keys,obj.ID,'IgnoreCase',true))
                % Connection to the same drone ID already exists
                check=true;
            elseif ~isempty(obj.IPAddress) && any(contains(obj.ConnectionMap.keys,obj.IPAddress,'IgnoreCase',true))
                % Connection to the same drone IP already exists
                check=true;
            elseif ~isempty(obj.ConnectionMap) && any(~cell2mat(obj.ConnectionMap.values)) && ~obj.StationMode
                % A connection already exists to a drone which is not in
                % station mode. Hence another Tello drone can't be
                % connected unless its a Tello EDU in station mode
                check=true;
            else
                check=false;
            end
        end

        function saveDroneProperties(obj)
            % Save the drone details once a connection is established to
            % the drone

            % Use the saved details to populate ryzelist output
            ryzeio.internal.Utility.updateActiveConnections(obj,'save');
            % Store the drone ID, IP Address and it's StationMode value
            % in the Map of connected drones
            initConnectionMap(obj);
        end

        function initConnectionMap(obj)
            % Initialize Connection Map with details of connected drones

            % Use the ID and IP address of each connected drone as its
            % key
            obj.ConnectionKey = char(sprintf("%s(%s)",obj.ID,obj.IPAddress));

            % Store the drone's connection key and it's StationMode value
            % in the MAP of connected drones
            obj.ConnectionMap(obj.ConnectionKey) = obj.StationMode;
        end

        function initDroneSensorConnection(obj)
            % Initialize a common connection to read sensor data of all
            % Tello drones

            try
                % Establish UDP connection to receive sensor data stream,
                % only for the first drone connection. The rest of the
                % drones will share this connection
                if obj.SensorConnection.Count == 0
                    % Initialize the connection to collect sensor data from
                    % all connected drones. Store the connection handle in
                    % a Map to share between all connected drones
                    obj.SensorConnection('SensorConnectionObj') = ryzeio.internal.SensorDataWarehouse();
                    % Start reading sensor data from all connected drones
                    startSensorDataCollection(obj.SensorConnection('SensorConnectionObj'),obj.DronePort,obj.HostPort);
                end

                % Initialize and start timer to read latest sensor data
                obj.SensorTimerObj = internal.IntervalTimer(0.1);
                addlistener(obj.SensorTimerObj, 'Executing', @(~, ~)obj.readSensorData);
                obj.SensorTimerObj.start();
            catch e
                throwAsCaller(e);
            end
        end

        function stopDroneTimers(obj)
            % Function to stop all the running timers that are updating the
            % drone properties

            % Stop timer and clear timer object used to read the sensor
            % data packets of the drone
            if(~isempty(obj.SensorTimerObj)  && isvalid(obj.SensorTimerObj))
                obj.SensorTimerObj.stop();
                obj.SensorTimerObj = [];
            end

            % Stop timer and clear timer object used to read command ACK from the drone
            if(~isempty(obj.CommandResponseTimerObj) && isvalid(obj.CommandResponseTimerObj))
                obj.CommandResponseTimerObj.stop();
                obj.CommandResponseTimerObj = [];
            end

            % Stop timer and clear timer object used to fetch the drone SSID
            if(~isempty(obj.DroneDataTimerObj) && isvalid(obj.DroneDataTimerObj))
                obj.DroneDataTimerObj.stop();
                obj.DroneDataTimerObj = [];
            end
        end

        function closeDroneConnections(obj)
            % Function to close all active connections to the drone

            % Close connection used to send commands and receive ACK
            if(~isempty(obj.CommandConnection) &&...
                    isvalid(obj.CommandConnection) && isOpen(obj.CommandConnection))
                % Stop video stream and close connection
                write(obj.CommandConnection, uint8('streamoff'));
                % Notify camera object about ryze object deletion
                unregister(obj.CommandConnection);
                close(obj.CommandConnection);
            end

            % Close Secondary connection used to fetch the drone SSID
            if(~isempty(obj.SecondaryDataConnection) &&...
                    isvalid(obj.SecondaryDataConnection) && isOpen(obj.SecondaryDataConnection))
                close(obj.SecondaryDataConnection);
            end
        end

        %% Functions to read and decode sensor data sent by drone to host port: 8890
        function readSensorData(obj)
            % Function to read the drone sensor data
            try
                if any(contains(obj.SensorConnection('SensorConnectionObj').SensorDataMap.keys,obj.IPAddress))
                    % The latest sensor data of all connected Tello drones
                    % is stored in a Map against their IP address.Read the
                    % sensor data for the required drone's IP address.
                    sensorData = obj.SensorConnection('SensorConnectionObj').SensorDataMap(obj.IPAddress);
                    % Extract the drone sensor values from the received
                    % sensor data
                    extractSensorData(obj, sensorData);
                end
            catch e
                stopDroneTimers(obj);
                throwAsCaller(e);
            end
        end

        function extractSensorData(obj, sensorData)
            % Function to extract individual sensor data from the packet
            % containing all drone sensor data

            try
                % Update battery level
                obj.BatteryLevel = str2double(extractBetween(sensorData,'bat:',';'));
                if(obj.BatteryLevel < 10)
                    lowBatteryWarning = ryzeio.internal.Utility.validateAlertState(obj.LowBattery, obj.BatteryLevel);
                    % Set the low battery flag
                    if(lowBatteryWarning)
                        obj.LowBattery = true;
                    end
                end

                % Update Attitude/Orientation Values
                Pitch = str2double(extractBetween(sensorData,'pitch:',';'));
                Roll = str2double(extractBetween(sensorData,'roll:',';'));
                Yaw = str2double(extractBetween(sensorData,'yaw:',';'));
                obj.Attitude.Value = [Yaw Pitch Roll];
                obj.Attitude.Time = datetime('now', 'TimeZone','local','Format','dd-MMM-uuuu HH:mm:ss.SS');

                % Update the Altitude/height of the drone
                obj.Altitude.Value = str2double(extractBetween(sensorData,';h:',';'));
                obj.Altitude.Time = datetime('now', 'TimeZone','local','Format','dd-MMM-uuuu HH:mm:ss.SS');

                % Update the speed of the drone
                vgx = str2double(extractBetween(sensorData,'vgx:',';'));
                vgy = str2double(extractBetween(sensorData,'vgy:',';'));
                vgz = str2double(extractBetween(sensorData,'vgz:',';'));
                obj.Speed.Value = [vgx vgy vgz];
                obj.Speed.Time = datetime('now', 'TimeZone','local','Format','dd-MMM-uuuu HH:mm:ss.SS');

                % Update the acceleration of the drone
                agx = str2double(extractBetween(sensorData,'agx:',';'));
                agy = str2double(extractBetween(sensorData,'agy:',';'));
                agz = str2double(extractBetween(sensorData,'agz:',';'));
                obj.Accel.Value = [agx agy agz];
                obj.Accel.Time = datetime('now', 'TimeZone','local','Format','dd-MMM-uuuu HH:mm:ss.SS');

                if(~obj.FirstPacketReceived)
                    % Make the 'FirstPacketReceived' true to indicate
                    % that the first packet of sensor data has arrived
                    obj.FirstPacketReceived = true;
                end
            catch e
                throwAsCaller(e);
            end
        end

        function basicMoveImpl(obj, direction, varargin)
            % Function to implement all basic move commands

            try

                % Validate input parameters of basic move APIs
                [distance, speed, blocking] = ryzeio.internal.Utility.validateBasicMoveParams(varargin{:}, obj.DistanceLimits, obj.SpeedLimits);

                % Validate the direction of movement
                direction = ryzeio.internal.Utility.validateMoveDirection(direction);

                % Convert distance to cm
                distance  = distance * 100;
                speed = speed * 100;

                % Set the timeout of the drone for the given distance and
                % speed. The default timeout might not give enough time for
                % the drone to move to a large distance and send response
                obj.CommandTimeout = distance/speed + ryzeio.internal.RyzeConstants.BufferTimeout;

                % Set speed and then send move command
                obj.CommandAck = false;
                command = char(sprintf("speed %d", round(speed)));
                write(obj.CommandConnection, uint8(command));
                waitForCommandAck(obj);

                obj.CommandAck = false;
                obj.CommandError = false;
                if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Hovering)))
                    obj.State = ryzeio.internal.DroneStateEnum.Flying;
                end

                command = char(sprintf("%s %d", direction, round(distance)));
                write(obj.CommandConnection, uint8(command));

                % Block MATLAB till ACK is received from the drone
                if(blocking)
                    try
                        waitForCommandAck(obj);
                    catch e
                        if(strcmpi(e.identifier, "MATLAB:ryze:general:commandError"))
                            if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Flying)))
                                obj.State = ryzeio.internal.DroneStateEnum.Hovering;
                            end
                        end

                        throwAsCaller(e);
                    end
                else
                    % Wait to see if there are any errors sent by the drone
                    pause(0.6);
                    if(obj.CommandError)
                        if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Flying)))
                            obj.State = ryzeio.internal.DroneStateEnum.Hovering;
                        end
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:commandError');
                    end
                end
            catch e
                throwAsCaller(e);
            end

            % Reset the timeout to the default value
            obj.CommandTimeout = ryzeio.internal.RyzeConstants.DefaultCommandTimeout;
        end

        %% Functions to read and decode drone status data sent by drone to Host port 8899
        function readDroneData(obj)
            % Function to read the drone status data from the Asyncio buffer

            try
                if(~isOpen(obj.SecondaryDataConnection))
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end

                try
                    droneData = read(obj.SecondaryDataConnection);
                catch
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end
                processDroneData(obj, droneData);
            catch e
                obj.DroneDataTimerObj.stop();
                obj.DroneDataTimerObj = [];
                throwAsCaller(e);
            end
        end

        function processDroneData(obj, droneData)
            % Process multiple chunks of drone status data received in single read operation

            try
                if(~isempty(droneData))
                    for i = 1:length(droneData)
                        decodeDroneData(obj, droneData(i).Data);
                    end
                end
            catch
                throwAsCaller(e);
            end
        end

        function decodeDroneData(obj, droneData)
            % Function to decode the drone SSID data sent by drone

            try
                if(17 == typecast([droneData(6) droneData(7)], 'uint16'))
                    obj.DetectedID = string(char(droneData(12:end-2)));
                end
            catch e
                throwAsCaller(e);
            end
        end
        %% Function to read command response from drone
        function readCommandResponseData(obj)
            % Function to read the drone ACK data from the Asyncio buffer

            try
                if(~isOpen(obj.CommandConnection))
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                end
                if(obj.CommandConnection.DataAvailable)
                    try
                        % Reset NoDataReceived counter
                        obj.NoDataReceivedCount = 0;
                        droneData = read(obj.CommandConnection);
                    catch
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionFailed');
                    end
                    processCommandResponseData(obj, droneData);
                elseif ~isempty(obj.NoDataReceivedCount) && ~ryzeio.internal.Utility.validateTestEnvironment
                    % If data is not available, check whether the drone got
                    % disconnected. NoDataReceivedCount is initialized
                    % only after connection is established to the drone and
                    % data is received.Hence avoid this check till
                    % NoDataReceivedCount is initialized. Also avoid this
                    % check for Mock Tello till g2528874 is resolved

                    % Increment the NoDataReceived counter and error after
                    % 30 retries (30*0.5s = 15s)
                    obj.NoDataReceivedCount = obj.NoDataReceivedCount + 1;
                    if(obj.NoDataReceivedCount >= 30)
                        obj.IsTelloConnected = false;
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionLost');
                    end
                end
            catch e
                stopDroneTimers(obj);
                throwAsCaller(e);
            end
        end

        function processCommandResponseData(obj, droneData)
            % Process multiple chunks of drone ACK data received in single read
            % operation

            try
                if(~isempty(droneData))
                    for i = 1:length(droneData)
                        decodeCommandResponseData(obj, droneData(i).Data);
                    end
                end
            catch e
                throwAsCaller(e);
            end
        end

        function decodeCommandResponseData(obj, droneData)
            % Function to decode the ACK data sent by drone

            try
                dataReceived = char(droneData);

                if(startsWith(dataReceived, 'ok','IgnoreCase',true))
                    % Drone acknowledges with response packet 'ok'. For
                    % station mode command it acknowledges with response
                    % 'OK, drone will reboot in 3s'
                    obj.CommandAck = true;
                    if(strcmpi(obj.State, char(ryzeio.internal.DroneStateEnum.Flying)))
                        obj.State = ryzeio.internal.DroneStateEnum.Hovering;
                    end
                elseif(contains(dataReceived, {'error','unknown command'}))
                    % Stay silent about errors thrown by drone when
                    % previous non-blocking command was not run to
                    % completion
                    if(~contains(dataReceived,{'error Auto land', 'error Not joystick'}))
                        obj.CommandError = true;
                    end
                end
            catch e
                throwAsCaller(e);
            end
        end

        function waitForCommandAck(obj)
            % Function to wait till an ACK is sent by the Tello drone in
            % response to a control command

            try
                obj.CommandError = false;
                t = tic;
                while(toc(t) < obj.CommandTimeout && ~obj.CommandAck)
                    if(obj.CommandError)
                        ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:commandError');
                    end
                    drawnow;
                end

                % Send error status if ACK is not received even after timeout
                if(~obj.CommandAck)
                    ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:commandFailed');
                end
            catch e
                throwAsCaller(e);
            end

        end

        function status = isTelloEDU(obj)
            % Function to check if the Tello drone connected is the EDU version
            % This can be verified by sending the command "mon" to enable
            % mission pad control. This command will return an ACK in Tello EDU
            % and "unknown command" in a standard Tello

            try
                status = false;
                obj.CommandAck = false;
                obj.CommandError = false;
                write(obj.CommandConnection, uint8('mon'));
                t = tic;
                while(toc(t) < obj.CommandTimeout && ~obj.CommandAck)
                    if(obj.CommandError)
                        status = false;
                        return;
                    end
                    drawnow;
                end
                % If a command ACK was received, it implies we have a Tello EDU
                % Turn off mission pad control
                if(obj.CommandAck)
                    status = true;
                    obj.CommandAck = false;
                    write(obj.CommandConnection, uint8('moff'));
                    t = tic;
                    while(toc(t) < obj.CommandTimeout && ~obj.CommandAck)
                        if(obj.CommandError)
                            return;
                        end
                        drawnow;
                    end
                end
            catch e
                throwAsCaller(e);
            end
        end

        function checkDroneConnection(obj)
            % Error out if the drone connection is not active
            if ~obj.IsTelloConnected
                ryzeio.internal.Utility.localizedError('MATLAB:ryze:general:connectionLost');
            end
        end
    end

    methods (Access = {?ryzeio.setup.internal.StationModeConfiguration}, Hidden)
        % Setup screen methods

        function enableStationMode(obj,ssid,password)
            enableStationModeImpl(obj,ssid,password)
        end
    end

    methods (Access = public, Hidden)
        % Disable and hide the methods below:

        % Hidden methods from the hgsetget super class.
        function res = eq(obj, varargin)
            res = eq@handle(obj, varargin{:});
        end

        function result = fieldnames(obj)
            result = obj.fieldnames@handle();
        end

        function result = fields(obj)
            result = obj.fields@handle();
        end

        function res = findobj(obj, varargin)
            res = findobj@handle(obj, varargin{:});
        end

        function res = findprop(obj, varargin)
            res = findprop@handle(obj, varargin{:});
        end

        function res = addlistener(obj, varargin)
            res = addlistener@handle(obj, varargin{:});
        end

        function res = notify(obj, varargin)
            res = notify@handle(obj, varargin{:});
        end

        % Hidden methods from the dynamic properties superclass
        function res = addprop(obj, varargin)
            res = addprop@dynamicprops(obj, varargin{:});
        end

        function result = permute(obj,varargin)
            [result] = obj.permute@handle(varargin{:});
        end

        function result = reshape(obj,varargin)
            [result] = obj.reshape@handle(varargin{:});
        end

        function result = transpose(obj)
            [result] = obj.transpose@handle();
        end

        % Unsupported functions
        function c = horzcat(varargin)
            try
                if (nargin == 1)
                    c = varargin{1};
                else
                    ryzeio.internal.Utility.throwUnsupportedError;
                end
            catch e
                throwAsCaller(e);
            end
        end

        function c = vertcat(varargin)
            try
                if (nargin == 1)
                    c = varargin{1};
                else
                    ryzeio.internal.Utility.throwUnsupportedError;
                end
            catch e
                throwAsCaller(e);
            end
        end

        function c = cat(varargin)
            try
                if (nargin > 2)
                    ryzeio.internal.Utility.throwUnsupportedError;
                else
                    c = varargin{2};
                end
            catch
                throwAsCaller(e);
            end
        end

        function ge(~, varargin)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end

        function gt(~, varargin)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end

        function le(~, varargin)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end

        function lt(~, varargin)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end

        function ne(~, varargin)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end

        function listener(~)
            try
                ryzeio.internal.Utility.throwUnsupportedError;
            catch e
                throwAsCaller(e);
            end
        end
    end

    methods(Access = protected)
        function displayScalarObject(obj)
            % Function for  custom object display

            header = getHeader(obj);
            disp(header);

            % Display all class properties
            showAllProperties(obj);

            % Allow for the possibility of a footer.
            footer = getFooter(obj);
            if ~isempty(footer)
                disp(footer);
            end
        end
    end

    methods(Hidden, Access = public)
        function showAllProperties(obj)
            % Function to display all properties of the ryze class

            fprintf('                   Name: "%s"\n', obj.Name);
            fprintf('                     ID: "%s"\n', obj.ID);
            fprintf('              IPAddress: "%s"\n', obj.IPAddress);
            fprintf('                  State: "%s"\n', char(obj.State));
            fprintf('           BatteryLevel: %d %%\n', obj.BatteryLevel);
            fprintf('            StationMode: %d\n', obj.StationMode);
            if ~isempty(obj.AvailableCameras)
                fprintf('       AvailableCameras: ["%s"]\n', obj.AvailableCameras(:));
            else
                fprintf('       AvailableCameras: []\n');
            end
            fprintf('\n');
        end

        function showFunctions(~)
            % Function to display all the functions available in the ryze
            % class

            fprintf('\n');

            fprintf('   takeoff\n');
            fprintf('   land\n');
            fprintf('   abort\n');
            fprintf('   moveforward\n');
            fprintf('   moveback\n');
            fprintf('   moveright\n');
            fprintf('   moveleft\n');
            fprintf('   moveup\n');
            fprintf('   movedown\n');
            fprintf('   move\n');
            fprintf('   turn\n');
            fprintf('   readSpeed\n');
            fprintf('   readHeight\n');
            fprintf('   readOrientation\n');
            fprintf('   flip\n');
            fprintf('   camera\n');

            fprintf('\n');
        end
    end
end

% LocalWords:  Tello Wi TELLO FPV SDK streamoff tello ACK EDU dm ZYX moveforward moveback moveright
% LocalWords:  moveleft ccw openh streamon addon Async ssid Asyncio Time mon moff MMM uuuu HH vgx
% LocalWords:  vgy vgz pingserver ryzelist
