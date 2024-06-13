classdef Tello_drone_control < matlab.System
    % Tello_drone_control Utilisez ce bloc pour connecter et contrôler le drone Tello.
    %
    % Cette classe inclut l'ensemble minimal de fonctions nécessaires
    % pour définir un objet System.

    % Propriétés publiques et modifiables
    properties
        droneIP = "127.0.0.1"; % Adresse IP du drone (modifiable)
    end

    % Propriétés privées
    properties (Access = private)
        drone;          % Objet drone
        cam;            % Caméra du drone
        imgFront;       % Image capturée de la caméra frontale
        imgDown;        % Image capturée de la caméra inférieure
        hasTakenOff;    % Statut du décollage
        previousRcEnabled = 0; % Flag pour détecter un changement dans l'activation du RC
        SizeImageFront = [720, 960, 3]; % Taille de l'image par défaut pour la caméra frontale
        SizeImageDown = [240, 320, 3]; % Taille de l'image par défaut pour la caméra inférieure
        prvRoundedRcSpeeds = [0; 0; 0; 0]; % Dernières valeurs de la commande RC
        CurrentVideoDirection = VideoDirection.Forward; % Direction de la caméra
        hasTakenOffStatus; % Statut du décollage pour la sortie
        lastCommandTime; % Temps du dernier décollage ou atterrissage
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Initialisation de la connexion avec le drone et la caméra
            if obj.droneIP == "127.0.0.1" || isempty(obj.droneIP)
                obj.drone = ryze("Tello");
            else
                obj.drone = ryze(obj.droneIP);
            end
            obj.cam = camera(obj.drone);
            obj.imgFront = zeros(obj.SizeImageFront, 'uint8');
            obj.imgDown = zeros(obj.SizeImageDown, 'uint8');
            obj.hasTakenOff = false; % Initialisation du statut de décollage
            obj.hasTakenOffStatus = 0; % Initialisation du statut de décollage pour la sortie
            obj.lastCommandTime = uint64(0); % Initialisation du temps de la dernière commande
        end

        function [imageFront, imageDown, Eulerangles, speedXYZ, batteryLevel, currentVideoDirection, hasTakenOffStatus] = stepImpl(obj, enableVideo, tkoff, landing, enableRC, rcspeeds, cmdVideoDirection, xMove, yMove, zMove, degree, movexyz, moverotate)
            % Gestion de la vidéo si activée
            if enableVideo
                image = snapshot(obj.cam); % Récupération de l'image de la caméra

                % Mise à jour de l'image capturée en fonction de la direction de la caméra
                if ~isempty(image)
                    if obj.CurrentVideoDirection == VideoDirection.Forward && ~any(size(image) - obj.SizeImageFront)
                        obj.imgFront = image;
                    elseif obj.CurrentVideoDirection == VideoDirection.Downward && ~any(size(image) - obj.SizeImageDown)
                        obj.imgDown = image;
                    end
                end

                % Commande de changement de direction de la caméra si nécessaire
                if obj.CurrentVideoDirection ~= cmdVideoDirection
                    switch_camera(obj.drone, uint16(cmdVideoDirection));
                    obj.CurrentVideoDirection = cmdVideoDirection; % Mise à jour de la direction actuelle
                end
            end

            % Assignation des images aux sorties
            imageFront = obj.imgFront;
            imageDown = obj.imgDown;

            % Gestion du décollage
            if tkoff && ~obj.hasTakenOff
                takeoff(obj.drone, "WaitUntilDone", false);
                obj.hasTakenOff = true;
                obj.lastCommandTime = tic;
            end

            % Gestion de l'atterrissage
            if landing && obj.hasTakenOff
                land(obj.drone, "WaitUntilDone", false);
                obj.hasTakenOff = false;
                obj.lastCommandTime = tic;
            end

            % Mise à jour du statut de décollage après 2 secondes
            if obj.lastCommandTime ~= 0 && toc(obj.lastCommandTime) > 2
                if obj.hasTakenOff
                    obj.hasTakenOffStatus = 1;
                else
                    obj.hasTakenOffStatus = 0;
                end
                obj.lastCommandTime = uint64(0); % Reset timer after status update
            end

            % % Déplacement du drone
            % if movexyz
            %     move(obj.drone, [xMove, yMove, zMove], "WaitUntilDone", false);
            % end
            % 
            % % Rotation du drone
            % if moverotate
            %     turn(obj.drone, deg2rad(degree), "WaitUntilDone", false);
            % end

            % Lecture des angles d'Euler et de la vitesse
            [Eulerangles, ~] = readOrientation(obj.drone);
            [speedXYZ, ~] = readSpeed(obj.drone);

            % Envoi de la commande RC si activée
            if enableRC
                roundedRcSpeeds = round(rcspeeds);
                if any(roundedRcSpeeds - obj.prvRoundedRcSpeeds)
                    rc(obj.drone, int16(roundedRcSpeeds(2)), int16(roundedRcSpeeds(1)), int16(roundedRcSpeeds(3)), int16(roundedRcSpeeds(4)));
                    obj.prvRoundedRcSpeeds = roundedRcSpeeds;
                end
            elseif obj.previousRcEnabled
                rc(obj.drone, 0, 0, 0, 0); % Arrêt de la commande RC si désactivée
                obj.prvRoundedRcSpeeds = [0; 0; 0; 0];
            end

            % Lecture du niveau de batterie
            batteryLevel = readBattery(obj.drone);

            % Renvoi de la direction actuelle de la caméra
            currentVideoDirection = uint16(obj.CurrentVideoDirection);

            % Mise à jour de l'état précédent du RC
            obj.previousRcEnabled = enableRC;

            % Assignation du statut de décollage à la sortie
            hasTakenOffStatus = obj.hasTakenOffStatus;
        end

        function resetImpl(obj)
            % Réinitialisation des propriétés internes
            obj.hasTakenOff = false;
            obj.hasTakenOffStatus = 0;
            obj.lastCommandTime = uint64(0);
        end

        function releaseImpl(obj)
            % Fermeture de la connexion à la caméra et au drone
            clear obj.cam;
            clear obj.drone;
        end

        %% Fonctions Simulink
        function [out1, out2, out3, out4, out5, out6, out7] = getOutputSizeImpl(obj)
            % Retourne la taille pour chaque port de sortie
            out1 = obj.SizeImageFront; % Taille de l'image frontale
            out2 = obj.SizeImageDown;  % Taille de l'image inférieure
            out3 = [1, 3]; % Angles ZYX
            out4 = [1, 3]; % Vitesse XYZ
            out5 = [1, 1]; % Niveau de batterie
            out6 = [1, 1]; % Direction actuelle de la vidéo
            out7 = [1, 1]; % Statut du décollage
        end

        function [out1, out2, out3, out4, out5, out6, out7] = getOutputDataTypeImpl(obj)
            % Retourne le type de données pour chaque port de sortie
            out1 = "uint8";    % Type de données pour l'image frontale
            out2 = "uint8";    % Type de données pour l'image inférieure
            out3 = 'double';   % Type de données pour les angles d'Euler
            out4 = 'double';   % Type de données pour la vitesse
            out5 = 'double';   % Type de données pour le niveau de batterie
            out6 = 'uint16';   % Type de données pour la direction de la caméra
            out7 = 'double';   % Type de données pour le statut du décollage
        end

        function [out1, out2, out3, out4, out5, out6, out7] = isOutputComplexImpl(obj)
            % Retourne false pour chaque port de sortie avec des données non complexes
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7 = false;
        end

        function [out1, out2, out3, out4, out5, out6, out7] = isOutputFixedSizeImpl(obj)
            % Retourne true pour chaque port de sortie avec une taille fixe
            out1 = true;  % Taille fixe pour l'image frontale
            out2 = true;  % Taille fixe pour l'image inférieure
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            out7 = true;
        end

        function num = getNumInputsImpl(obj)
            % Définissez le nombre d'entrées du système
            num = 6; % Nombre d'entrées attendu
        end

        function num = getNumOutputsImpl(obj)
            % Définissez le nombre de sorties du système
            num = 7; % Nombre de sorties attendu
        end
    end
end
