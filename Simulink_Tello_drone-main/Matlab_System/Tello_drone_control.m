classdef Tello_drone_control < matlab.System
    % Tello_drone_control Utilisez ce bloc pour connecter et contrôler le drone Tello.
    %
    % Cette classe inclut l'ensemble minimal de fonctions nécessaires
    % pour définir un objet System.

    % Propriétés publiques et modifiables
    properties
        droneIP = "127.0.0.1";
    end

    % Propriétés privées
    properties (Access = private)
        drone;          % Objet drone
        cam;            % Caméra du drone
        imgFront;       % Image capturée de la caméra frontale
        imgDown;        % Image capturée de la caméra inférieure
        hasTakenOff;    % Statut du décollage
        previousRcEnabled = 0; % Flag pour détecter un front montant/descendant sur le RC Enabled
        SizeImageFront = [720, 960, 3]; % Taille de l'image par défaut pour la caméra frontale
        SizeImageDown = [480, 640, 3]; % Taille de l'image par défaut pour la caméra inférieure
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Initialisez la connexion avec le drone et la caméra
            if obj.droneIP == "127.0.0.1" || isempty(obj.droneIP)
                obj.drone = ryze("Tello");
            else
                obj.drone = ryze(obj.droneIP);
            end
            obj.cam = camera(obj.drone);
            obj.imgFront = zeros(obj.SizeImageFront, 'uint8');
            obj.imgDown = zeros(obj.SizeImageDown, 'uint8');
            obj.hasTakenOff = false; % Initialisation du statut de décollage
        end

        function [imageFront, imageDown, Eulerangles, speedXYZ, accelXYZ] = stepImpl(obj, Slam, tkoff, landing, xMove, yMove, zMove, degree, movexyz, moverotate,enableRC,rcspeeds, downcam)
            % Capturez l'image si le commutateur Slam est activé
            if Slam == 1
                if downcam == 1
                    switch_camera(obj.drone, "down");
                    image = snapshot(obj.cam);
                    if ~isempty(image)
                        obj.imgDown = image;
                    end
                else
                    switch_camera(obj.drone, "front");
                    image = snapshot(obj.cam);
                    if ~isempty(image)
                        obj.imgFront = image;
                    end
                end
            end
            imageFront = obj.imgFront;
            imageDown = obj.imgDown;

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
            % Commande RC
            if enableRC == 1
                rc(obj.drone, rcspeeds(2), rcspeeds(1), rcspeeds(3), rcspeeds(4));
            end

            % Envoie d'une commande RC (0,0,0,0) lorsque l'on désactive le
            % RC
            if enableRC == 0 && obj.previousRcEnabled == 1
                rc(obj.drone, 0, 0, 0, 0);
            end

            [accelXYZ, ~] = readAccel(obj.drone);

            % Store previous value
            obj.previousRcEnabled = enableRC;

           
            % Lecture des angles d'Euler et de la vitesse

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
        function [out1, out2, out3, out4,out5] = getOutputSizeImpl(obj)
            % Retourne la taille pour chaque port de sortie
            out1 = obj.SizeImageFront; % Taille dynamique de l'image frontale
            out2 = obj.SizeImageDown;  % Taille dynamique de l'image inférieure
            out3 = [1, 3]; % Angles ZYX
            out4 = [1, 3]; % Vitesse XYZ
            out5 = [1, 3]; % Accélération XYZ
        end

        function [out1, out2, out3, out4, out5] = getOutputDataTypeImpl(obj)
            % Retourne le type de données pour chaque port de sortie
            out1 = "uint8";
            out2 = "uint8";
            out3 = 'double';
            out4 = 'double';
            out5 = 'double';

        end

        function [out1, out2, out3, out4, out5] = isOutputComplexImpl(obj)
            % Retourne false pour chaque port de sortie avec des données non complexes
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
        end

        function [out1, out2, out3, out4, out5] = isOutputFixedSizeImpl(obj)
            % Retourne true pour chaque port de sortie avec une taille fixe
            out1 = true; % Taille dynamique pour l'image frontale
            out2 = true; % Taille dynamique pour l'image inférieure
            out3 = true;
            out4 = true;
            out5 = true;
        end

        function num = getNumInputsImpl(obj)
            % Définissez le nombre d'entrées du système
            num = 12; % Slam, takeoff, land et les valeurs de déplacement
        end

        function num = getNumOutputsImpl(obj)
            % Définissez le nombre de sorties du système
            num = 4; % Image frontale, image inférieure, angles Euler, vitesse XYZ
        end
    end
end
