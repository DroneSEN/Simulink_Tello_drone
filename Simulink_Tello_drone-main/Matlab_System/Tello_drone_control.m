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
        imgBoth;        % Image capturée des deux caméras
        hasTakenOff;    % Statut du décollage
        previousRcEnabled = 0; % Flag pour détecter un front montant/descendant sur le RC Enabled
        SizeImageFront = [720, 960, 3]; % Taille de l'image par défaut pour la caméra frontale
        SizeImageDown = [240, 320, 3]; % Taille de l'image par défaut pour la caméra inférieure
        SizeImageMix = [1000, 1000, 3]; % Taille de l'image avec les deux vidéos
        prvRoundedRcSpeeds = [0; 0; 0; 0]; % Dernières valeurs de la commande RC
        CurrentVideoDirection = VideoDirection.Forward; % Direction de la caméra
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
            obj.imgBoth = zeros(obj.SizeImageFront, 'uint8');
            obj.hasTakenOff = false; % Initialisation du statut de décollage

            % Activation des mission pads
            % activateMissionPad(obj.drone)
        end

        function [imageFront, imageDown, imageBoth, Eulerangles, speedXYZ, accelXYZ, batteryLevel, currentVideoDirection] = stepImpl(obj, enableVideo, tkoff, landing, xMove, yMove, zMove, degree, movexyz, moverotate,enableRC,rcspeeds, cmdVideoDirection)
            % Gestion de la video si activé
            if enableVideo == 1

                % Récupération de l'image
                image = snapshot(obj.cam);

                % On recopie l'image dans l'attribut associée
                if ~isempty(image)
                    switch obj.CurrentVideoDirection
                        case VideoDirection.Forward
                            if ~any(size(image) - obj.SizeImageFront)
                                obj.imgFront = image;
                            end
                        case VideoDirection.Downward
                            if ~any(size(image) - obj.SizeImageDown)
                                % Si la taille est correcte, on copie
                                % l'image
                                obj.imgDown = image;
                            end
                        case VideoDirection.Both
                            % Split de l'image
                            s = size(image);
                            obj.imgBoth(1:s(1), 1:s(2), :) = image;
                    end
                end
                
                % Si la direction de la caméra change depuis le dernier
                % cycle, on envoit une commande de changement
                if obj.CurrentVideoDirection ~= cmdVideoDirection
                    
                    switch_camera(obj.drone, uint16(cmdVideoDirection));

                    % On sauvegarde la direction actuelle
                    obj.CurrentVideoDirection = cmdVideoDirection;
                end
            end
    
            % Output the data
            imageFront = obj.imgFront;
            imageDown = obj.imgDown;
            imageBoth = obj.imgBoth;

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

            % Envoie de la commande RC
            if enableRC == 1
                roundedRcSpeeds = round(rcspeeds);

                % On envoie la commande si elle diffère de la précédente 
                if any(roundedRcSpeeds-obj.prvRoundedRcSpeeds)
                    rc(obj.drone, int16(roundedRcSpeeds(2)), ...
                                  int16(roundedRcSpeeds(1)), ...
                                  int16(roundedRcSpeeds(3)), ...
                                  int16(roundedRcSpeeds(4)));
                    obj.prvRoundedRcSpeeds = roundedRcSpeeds;
                end
            end

            % Envoie d'une commande RC (0,0,0,0) lorsque l'on désactive le
            % RC
            if enableRC == 0 && obj.previousRcEnabled == 1
                rc(obj.drone, 0, 0, 0, 0);
                obj.prvRoundedRcSpeeds = [0;0;0;0];
            end

            [accelXYZ, ~] = readAccel(obj.drone);

            % Store previous value
            obj.previousRcEnabled = enableRC;
           
            % Lecture des angles d'Euler et de la vitesse

            % Renvoie de la batterie
            batteryLevel = readBattery(obj.drone);

            % Renvoie de la direction de caméra
            currentVideoDirection = uint16(obj.CurrentVideoDirection);

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
        function [out1, out2, out3, out4,out5, out6, out7, out8] = getOutputSizeImpl(obj)
            % Retourne la taille pour chaque port de sortie
            out1 = obj.SizeImageFront; % Taille dynamique de l'image frontale
            out2 = obj.SizeImageDown;  % Taille dynamique de l'image inférieure
            out3 = obj.SizeImageMix;  % Taille dynamique des deux images
            
            out4 = [1, 3]; % Angles ZYX
            out5 = [1, 3]; % Vitesse XYZ
            out6 = [1, 3]; % Accélération XYZ
            out7 = [1, 1]; % Niveau de batterie

            out8 = [1, 1]; % Direction actuelle de la vidéo
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8] = getOutputDataTypeImpl(obj)
            % Retourne le type de données pour chaque port de sortie
            out1 = "uint8";
            out2 = "uint8";
            out3 = "uint8";

            out4 = 'double';
            out5 = 'double';
            out6 = 'double';
            out7 = 'double';

            out8 = "uint16"; % Direction actuelle de la vidéo
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8] = isOutputComplexImpl(obj)
            % Retourne false pour chaque port de sortie avec des données non complexes
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7 = false;
            out8 = false;
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8] = isOutputFixedSizeImpl(obj)
            % Retourne true pour chaque port de sortie avec une taille fixe
            out1 = true; % Taille dynamique pour l'image frontale
            out2 = false; % Taille dynamique pour l'image inférieure
            out3 = false;
            out4 = true;
            out5 = true;
            out6 = true;
            out7 = true;
            out8 = true;
        end

        function num = getNumInputsImpl(obj)
            % Définissez le nombre d'entrées du système
            num = 12; % Slam, takeoff, land et les valeurs de déplacement
        end

        function num = getNumOutputsImpl(obj)
            % Définissez le nombre de sorties du système
            num = 8; % Images (frontales, inférieures, mix), angles Euler, vitesse XYZ, direction caméra
        end
    end
end
