classdef Optilink < matlab.System
    %OPTILINK Summary of this class goes here

    properties
        Server
    end

    methods
        function obj = Optilink(hostip, clientip)
            obj.Server = natnet();
            obj.Server.HostIP = hostip;
            obj.Server.ClientIP = clientip;
        end

        function connect(obj)
            obj.Server.connect();
        end

        function disconnect(obj)
            obj.Server.disconnect();
        end

        function drones = fetchOptitrackGroundTruth(obj)
            %fetchOptitrackGroundTruth Retrieve the last real position of the drone
            % Data format: [1x8]
            % - timestamp
            % - Tx, ty, tz
            % - qx, qy, qz, qw
            % Error = 1 when an error occured, with position equal to [0, 0, 0]
            

            if obj.Server.IsConnected()
                frame = obj.Server.getFrame();

                % Retrieve the number of rigid bodies
                rigidBodies = frame.nRigidBodies;

                drones = cell(1, rigidBodies);
                drone_matrice = zeros(4,4);

                for i = 1:rigidBodies
                    rigidbody = frame.RigidBodies(i);
                    t = [rigidbody.x rigidbody.y rigidbody.z];
                    q = [rigidbody.qx rigidbody.qy rigidbody.qz rigidbody.qw];
                    drone_matrice = Optilink.quaternionTranslationToTForm(q, t);
                    drones{i} = drone_matrice;
                end
            else
                drones = cell(1, 0);
                display("Not connected");
            end
        end
    end

    methods (Static)
        function TForm = quaternionTranslationToTForm(q, t)
            % Normalize quaternion
            q = q / norm(q);
        
            % Construct rotation matrix from quaternion
            R = quat2rotm(q);
        
            % Construct homogeneous transformation matrix
            tmp = eye(4); % Initialize as identity matrix
            tmp(1:3, 1:3) = R; % Insert rotation matrix
            tmp(1:3, 4) = t; % Insert translation vector
            TForm = tmp;
        end
    end
end

