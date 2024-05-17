classdef Tello_drone_control < matlab.System
    % TelloDrone Use this block to connect to the drone
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properopenopeties
    properties
        droneIP = '127.0.0.1';
    end


    properties (Access = private)

        drone;
        cam;
        img;
        hasTakenOff;
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Connectez-vous à la caméra du drone
            if isempty(obj.droneIP)
                obj.drone = ryze("Tello");
            else
                obj.drone = ryze("Tello");
            end
            obj.cam = camera(obj.drone);
            obj.img = zeros(720,960,3,'uint8');
            obj.hasTakenOff = false; % Initialisation de hasTakenOff
        end


        function [imgresize, Eulerangles, speedXYZ] = stepImpl(obj, Slam, tkoff, landing, xMove, yMove, zMove, degree, movexyz, moverotate)
            % Capturez l'image uniquement lorsque le commutateur est activé
            if Slam == 1
                image = snapshot(obj.cam);
                imgresize = imresize(image, 0.5);
                if isempty(image)
                    imgresize = obj.imgresize;
                else
                    obj.img = imgresize;
                end
            else
                imgresize = obj.img; % Retournez l'image précédente si le commutateur est désactivé
            end
            
            
            % Vérifiez si la commande de décollage est activée
            if tkoff == 1 && ~obj.hasTakenOff % Assurez-vous que le drone n'a pas encore décollé
                takeoff(obj.drone, "WaitUntilDone", false);
                obj.hasTakenOff = true; % Mettez à jour le statut du décollage
            end

            % Vérifiez si la commande d'atterrissage est activée
            if landing == 1 && obj.hasTakenOff % Assurez-vous que le drone n'a pas encore atterri
                land(obj.drone, "WaitUntilDone", false);
                obj.hasTakenOff = false; % Mettez à jour le statut de l'atterrissage
            end
            
            % Contrôlez le drone avec les valeurs de déplacement fournies
            if movexyz == 1 
                moveValues = [xMove, yMove, zMove];
                move(obj.drone, moveValues, "WaitUntilDone", false); 
            end

            if moverotate == 1
                rotatevalues = degree;
                turn(obj.drone,deg2rad(rotatevalues), "WaitUntilDone", false);
            end
                
            [Eulerangles, ~] = readOrientation(obj.drone);
            [speedXYZ,~] = readSpeed(obj.drone);
        end

        function resetImpl(obj)
            % Réinitialisez les propriétés internes
        end

        function releaseImpl(obj)
            % Fermez la connexion à la caméra du drone
            clear obj.cam;
            clear obj.drone;
        end

        %% Simulink functions
        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Retourne la taille pour chaque port de sortie
            out1= [360 480 3];
            out2 = [1 3]; % 3 valeurs d'angles ZYX
            out3 = [1 3]; % 3 valeurs vitesse XYZ
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(obj)
            % Retourne le type de données pour chaque port de sortie
            out1 = "uint8";
            out2 = 'double';
            out3 = 'double';
        end

        function [out1, out2, out3] = isOutputComplexImpl(obj)
            % Retourne true pour chaque port de sortie avec des données complexes
            out1 = false;
            out2 = false;
            out3 = false;
        end

        function [out1, out2, out3] = isOutputFixedSizeImpl(obj)
            % Retourne true pour chaque port de sortie avec une taille fixe
            out1 = true;
            out2 = true;
            out3 = true;
        end

        function num = getNumInputsImpl(obj)
            % Définissez le nombre d'entrées du système
            num = 9; % Le commutateur, takeoff, land et les valeurs de déplacement en entrée
        end
    end
end
