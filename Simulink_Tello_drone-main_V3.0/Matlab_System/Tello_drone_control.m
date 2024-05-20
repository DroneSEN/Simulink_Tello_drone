classdef Tello_drone_control < matlab.System
    % Tello_drone_control Utilisez ce bloc pour connecter et contrôler le drone Tello.
    %
    % Cette classe inclut l'ensemble minimal de fonctions nécessaires
    % pour définir un objet System.

    % Propriétés publiques et modifiables
    properties
        droneIP = '127.0.0.1';
    end

    % Propriétés privées
    properties (Access = private)
        drone;          % Objet drone
        cam;            % Caméra du drone
        img;            % Image capturée
        hasTakenOff;    % Statut du décollage
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Initialisez la connexion avec le drone et la caméra
            obj.drone = ryze("Tello");
            obj.cam = camera(obj.drone);
            obj.img = zeros(720, 960, 3, 'uint8');
            obj.hasTakenOff = false; % Initialisation du statut de décollage
        end

        function [imgresize, Eulerangles, speedXYZ] = stepImpl(obj, Slam, tkoff, landing, xMove, yMove, zMove, degree, movexyz, moverotate)
            % Capturez l'image si le commutateur Slam est activé
            image = snapshot(obj.cam);
            if Slam == 1
                if ~isempty(image)
                    obj.img = image;
                end
            end
            imgresize = obj.img;

            % Gestion du décollage
            if tkoff == 1 && ~obj.hasTakenOff
                takeoff(obj.drone, "WaitUntilDone", false);
                obj.hasTakenOff = true;
            end

            % Gestion de l'atterrissage
            if landing == 1 && obj.hasTakenOff
                land(obj.drone, "WaitUntilDone", false);
                obj.hasTakenOff = false;
            end

            % Déplacement du drone
            if movexyz == 1 
                moveValues = [xMove, yMove, zMove];
                move(obj.drone, moveValues, "WaitUntilDone", false);
            end

            % Rotation du drone
            if moverotate == 1
                turn(obj.drone, deg2rad(degree), "WaitUntilDone", false);
            end

            % Lecture des angles d'Euler et de la vitesse
            [Eulerangles, ~] = readOrientation(obj.drone);
            [speedXYZ, ~] = readSpeed(obj.drone);
        end

        function resetImpl(obj)
            % Réinitialisez les propriétés internes
            obj.hasTakenOff = false;
        end

        function releaseImpl(obj)
            % Fermez la connexion à la caméra et au drone
            clear obj.cam;
            clear obj.drone;
        end

        %% Fonctions Simulink
        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Retourne la taille pour chaque port de sortie
            out1 = [720, 960, 3];
            out2 = [1, 3]; % Angles ZYX
            out3 = [1, 3]; % Vitesse XYZ
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(obj)
            % Retourne le type de données pour chaque port de sortie
            out1 = "uint8";
            out2 = 'double';
            out3 = 'double';
        end

        function [out1, out2, out3] = isOutputComplexImpl(obj)
            % Retourne false pour chaque port de sortie avec des données non complexes
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
            num = 9; % Slam, takeoff, land et les valeurs de déplacement
        end
    end
end
