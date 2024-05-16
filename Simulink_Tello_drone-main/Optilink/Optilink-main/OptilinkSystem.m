classdef OptilinkSystem < matlab.System
    % OptilinkSystem Retrieve Optitrack data in Simulink
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
        HostIP (1, :) char = '127.0.0.1'
        ClientIP (1, :) char = '127.0.0.1'
    end

    % Pre-computed constants or internal states
    properties (Access = private)
        Server
        LatestCorrectPos
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants

            % Initialize variables
            obj.LatestCorrectPos = zeros(4,4);

            obj.Server = natnet();
            obj.Server.HostIP = obj.HostIP;
            obj.Server.ClientIP = obj.ClientIP;

            % Connect to the server
            obj.Server.connect();

            if obj.Server.IsConnected() == 1
                fprintf("Connected to Optitrack server!")
            else
                error('Could not connect to Optitrack server')
            end
        end

        function [drones, connected] = stepImpl(obj)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            if obj.Server.IsConnected() == 1
                frame = obj.Server.getFrame();

                % Retrieve the number of rigid bodies
                rigidBodies = frame.nRigidBodies;

                drone_matrice = zeros(4,4);
                
                if rigidBodies > 0
                    rigidbody = frame.RigidBodies(1);
                    t = [ rigidbody.x rigidbody.y rigidbody.z];
                    q = [rigidbody.qx rigidbody.qy rigidbody.qz rigidbody.qw];
                    drone_matrice = Optilink.quaternionTranslationToTForm(q, t);
                end
                obj.LatestCorrectPos = drone_matrice;
                drones = drone_matrice;
                connected = 1;
            else
                drones = obj.LatestCorrectPos;
                connected = 0;
            end
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
            % Disconnect from the server
            % obj.Server.disconnect();
        end

        function [out1,out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [4 4];
            out2 = [1 1];
        end

        function [out1,out2] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
        end

        function [out1,out2] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = 'double';
            out2 = 'double';
        end

        function [out1, out2] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
        end

        function [name1, name2] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name1 = 'Drones data';
            name2 = 'Connected';
        end

    end
end
