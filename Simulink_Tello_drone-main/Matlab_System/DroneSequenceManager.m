classdef DroneSequenceManager < matlab.System
    % Le DroneSequenceManager est un système de contrôle basé sur une machine d'état, implémenté dans Simulink, pour gérer les séquences opérationnelles d'un drone. Ce système est conçu pour assurer une gestion fluide et sécurisée des différentes phases de vol et des transitions en fonction des commandes et signaux d'entrée.

    % Public, tunable properties
    properties
        % Position initiale
        HomePosition = [0, 1.5, 0];
    end

    % Pre-computed constants or internal states
    properties (Access = private)
        % Etat actuel du drone - Idle à l'initialization
        CurrentState = DroneState.Idle;

        % Compteur de temps passé en état actuel
        % Ce timer doit être reset après chaque changement d'état
        TimeCounter = 0;

        % Temps précédent pour calculer le sample time
        PrvTime = 0;

        % Commandes
        TakeoffCmd = 0;
        LandCmd = 0;
        EnableRcCmd = 0;
        RefPosition = [0, 0, 0];
        EnableVideoCmd = 0;
        EnableSlamCmd = 0;
        VideoDirectionCmd = VideoDirection.Downward;

        % Source des données de position
        PosDataSrc = PositionDataSource.None;

        % Constantes

        % Délai maximal d'initialisation du drone (en secondes)
        STARTUP_MAX_DELAY_S = 5;

        % Délai maximal de régulation de la position initiale (en secondes)
        POS_REGULATION_MAX_DELAY_S = 10;
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            % Etat par défaut
            obj.CurrentState = DroneState.Idle;
            obj.TimeCounter = 0;
            obj.PrvTime = 0;
            
            % Commandes par défaut
            obj.TakeoffCmd = 0;
            obj.LandCmd = 0;
            obj.RefPosition = [0 0 0];
            obj.EnableVideoCmd = 0;
            obj.VideoDirectionCmd = VideoDirection.Downward;
        end

        function [takeoffCmd, landCmd, enableRcCmd, enableVideoCmd, enableSlamCmd, videoDirectionCmd, refPosition, timeCounter, state, posDataSrc] = stepImpl(obj,time, takeOffSignal, startPatrolingSignal, abortSignal, tookOffSignal, landedSignal, frontCamFrameReceived, downCamFrameReceived, waypointToFollow, finishedFollow)
            % Récupération du sample time
            sampleTime = time - obj.PrvTime;
            obj.TimeCounter = obj.TimeCounter + sampleTime;
            obj.PrvTime = time;

            % Sorties par défaut
            timeCounter = obj.TimeCounter;

            % Machine d'état pour contrôler le drone
            switch obj.CurrentState
                case DroneState.Idle
                    % Attente de la commande de démarrage
                    if takeOffSignal == 1
                        obj.CurrentState = DroneState.Startup;
                        obj.TimeCounter = 0;
                    end
                
                case DroneState.Startup
                    % Initialisation du drone
                    % - Activation du flux vidéo
                    % - Modification de la source des données position : Aruco
                    % - Activation de la caméra vers le bas
                    % - Attente de la première frame

                    % Modification de la source des données de position
                    % TODO : Change to Aruco
                    obj.PosDataSrc = PositionDataSource.Aruco;
                    
                    % Activation du flux vidéo
                    obj.EnableVideoCmd = 1;

                    % Activation de la caméra vers le bas
                    obj.VideoDirectionCmd = VideoDirection.Downward;

                    % Attente de la première frame
                    if downCamFrameReceived == 1
                        obj.CurrentState = DroneState.Takeoff;
                        obj.TimeCounter = 0;
                    end

                case DroneState.Takeoff
                    % Décollage du drone
                    obj.TakeoffCmd = 1;

                    % Source des données de position : Aruco
                    obj.PosDataSrc = PositionDataSource.Aruco;

                    % Transition vers la régulation de la position initiale
                    % Si le signal de takeoff est reçu
                    if tookOffSignal == 1
                        obj.CurrentState = DroneState.HomePosition;
                        obj.TimeCounter = 0;

                        % Réinitialiser la commande de décollage
                        obj.TakeoffCmd = 0;
                    end
                
                case DroneState.HomePosition
                    % Régulation de la position initiale

                    % Source des données de position : Aruco
                    % TODO : Change to Aruco
                    obj.PosDataSrc = PositionDataSource.Aruco;

                    % Waypoint de la position initiale
                    obj.RefPosition = obj.HomePosition;

                    % Asservissement sur la position initiale
                    obj.EnableRcCmd = 1;

                    % Transition vers l'initialisation de la patrouille
                    % - Lors de la réception du signal
                    % - Ou après un certain délai
                    % - Lorsque le drone est en position
                    % if startPatrolingSignal == 1 || obj.TimeCounter > obj.POS_REGULATION_MAX_DELAY_S
                    if startPatrolingSignal == 1
                        obj.CurrentState = DroneState.InitPatrolling;
                        obj.TimeCounter = 0;

                        % Réinitialiser la commande de décollage
                        obj.TakeoffCmd = 0;
                    end

                case DroneState.InitPatrolling
                    % Initialisation de la patrouille
                    % - Activation de la caméra avant
                    % - Activation du SLAM
                    obj.VideoDirectionCmd = VideoDirection.Forward;
                    obj.EnableSlamCmd = 1;

                    if frontCamFrameReceived == 1
                        % Transition vers l'état de patrouille
                        obj.CurrentState = DroneState.Patrolling;
                        obj.TimeCounter = 0;
                    end

                case DroneState.Patrolling
                    % Source des données de position : Optitrack
                    % TODO : Modifier la source vers SensorFusion
                    obj.PosDataSrc = PositionDataSource.Optitrack;

                    % Suivi des points de passage
                    obj.RefPosition = waypointToFollow;
                    obj.EnableRcCmd = 1;

                    if finishedFollow == 1
                        % Transition vers l'état d'atterrissage
                        obj.CurrentState = DroneState.Landing;
                        obj.TimeCounter = 0;
                        obj.RefPosition = obj.HomePosition;
                    end

                case DroneState.Landing
                    % Atterrissage du drone
                    obj.LandCmd = 1;

                    % Désactivation des commandes RC
                    obj.EnableRcCmd = 0;

                    % Source des données de position : Optitrack
                    % TODO : Modifier la source vers SensorFusion
                    obj.PosDataSrc = PositionDataSource.Optitrack;

                    % Transition vers l'état Landed
                    % Si le signal de landing est reçu
                    if landedSignal == 1
                        obj.CurrentState = DroneState.Landed;
                        obj.TimeCounter = 0;
                    end
                
                case DroneState.Landed
                    % Drone a atterri et est inactif

                    if takeOffSignal == 1
                        % Transition vers l'état de démarrage
                        obj.CurrentState = DroneState.Startup;
                        obj.TimeCounter = 0;
                    end
                
                case DroneState.Aborting
                    % Abandon de la mission
                    % - Atterrissage immédiat
                    % - Désactivation des caméras
                    % - Désactivation des commandes RC
                    obj.TakeoffCmd = 0;
                    obj.EnableRcCmd = 0;
                    obj.EnableVideoCmd = 0;
                    obj.EnableSlamCmd = 0;
                    obj.LandCmd = 1;

                    % Transition vers l'état Landed
                    % Si le signal de landing est reçu
                    if landedSignal == 1
                        obj.CurrentState = DroneState.Landed;
                        obj.TimeCounter = 0;
                    end

                otherwise
                    % Autres états non gérés
                    obj.CurrentState = DroneState.Idle;
            end

            % En cas d'urgence, atterrissage immédiat
            if abortSignal == 1 && obj.CurrentState ~= DroneState.Aborting
                obj.CurrentState = DroneState.Aborting;
                obj.TimeCounter = 0;
            end

            % Sorties des commandes
            takeoffCmd = obj.TakeoffCmd;
            % takeoffCmd = 0; % TODO : Remove
            landCmd = obj.LandCmd;
            enableRcCmd = obj.EnableRcCmd;
            % enableRcCmd = 0; % TODO : Remove
            refPosition = obj.RefPosition;
            enableVideoCmd = obj.EnableVideoCmd;
            enableSlamCmd = obj.EnableSlamCmd;
            videoDirectionCmd = obj.VideoDirectionCmd;
            posDataSrc = obj.PosDataSrc;

            % Sortie de l'état actuel
            state = obj.CurrentState;
            
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
            obj.CurrentState = DroneState.Idle;
        end

        function [takeoffCmd, landCmd, enableRcCmd, enableVideoCmd, enableSlamCmd, videoDirectionCmd, refPosition, timeCounter, state, posDataSrc] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            takeoffCmd          = 'double';
            landCmd             = 'double';
            enableRcCmd         = 'double';
            enableVideoCmd      = 'double';
            enableSlamCmd       = 'double';
            videoDirectionCmd   = 'VideoDirection';
            refPosition         = 'double';
            timeCounter         = 'double';
            state               = 'DroneState';
            posDataSrc          = 'PositionDataSource';
        end

        % is output fixed size
        function [takeoffCmd, landCmd, enableRcCmd, enableVideoCmd, enableSlamCmd, videoDirectionCmd, refPosition, timeCounter, state, posDataSrc] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            takeoffCmd          = true;
            landCmd             = true;
            enableRcCmd         = true;
            enableVideoCmd      = true;
            enableSlamCmd       = true;
            videoDirectionCmd   = true;
            refPosition         = true;
            timeCounter         = true;
            state               = true;
            posDataSrc          = true;
        end

        function [takeoffCmd, landCmd, enableRcCmd, enableVideoCmd, enableSlamCmd, videoDirectionCmd, refPosition, timeCounter, state, posDataSrc] = getOutputSizeImpl(~)
            % Return size for each output port
            takeoffCmd          = [1, 1];
            landCmd             = [1, 1];
            enableRcCmd         = [1, 1];
            enableVideoCmd      = [1, 1];
            enableSlamCmd       = [1, 1];
            videoDirectionCmd   = [1, 1];
            refPosition         = [1, 3];
            timeCounter         = [1, 1];
            state               = [1, 1];
            posDataSrc          = [1, 1];
        end

        function [takeoffCmd, landCmd, enableRcCmd, enableVideoCmd, enableSlamCmd, videoDirectionCmd, refPosition, timeCounter, state, posDataSrc] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            takeoffCmd          = false;
            landCmd             = false;
            enableRcCmd         = false;
            enableVideoCmd      = false;
            enableSlamCmd       = false;
            videoDirectionCmd   = false;
            refPosition         = false;
            timeCounter         = false;
            state               = false;
            posDataSrc          = false;
        end
        
    end
end
