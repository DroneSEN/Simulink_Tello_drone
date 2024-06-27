classdef MonoVisualSLAMSystem < matlab.System
    % MonoVisualSLAMSystem Algorithme de SLAM visuel monoculaire

    % Propriétés publiques, ajustables depuis Simulink
    properties(Nontunable)
        % FocalLength Longueur focale de la caméra
        FocalLength = [366.7411, 362.9497];
        % PrincipalPoint Centre principal de la caméra
        PrincipalPoint = [252.67, 206.61];
        % ImageSize Taille de l'image
        ImageSize = [360, 480];
        
        % ScaleFactor Facteur de mise à l'échelle pour la pyramide d'images
        ScaleFactor = 1.2;
        
        % NumLevels Nombre de niveaux de la pyramide
        NumLevels = 5;
        
        % MaxNumPoints Nombre maximum de points de caractéristiques
        MaxNumPoints = 1000;
        
        % SkipMaxFrames Nombre maximum de trames à sauter
        SkipMaxFrames = 20;
        
        % TrackFeatureRange Plage de suivi des caractéristiques
        TrackFeatureRange = [50, 120];
        
        % LoopClosureThreshold Seuil de fermeture de boucle
        LoopClosureThreshold = 50;
        
        % Verbose Mode verbeux
        Verbose = false;
        
        % ThreadLevel Niveau de parallélisme
        ThreadLevel = 8;
    end

    % Constantes pré-calculées
    properties(Access = private)
        VslamObj     % Objet SLAM visuel
        Pose (4,4) double % Matrice de pose de la caméra
    end

    methods
        % Constructeur
        function obj = MonoVisualSLAMSystem(varargin)
            % Supporte les arguments de type name-value lors de la construction de l'objet
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods(Access = protected)
        %% Fonctions communes
        function setupImpl(obj)
            % Effectue les calculs nécessaires une seule fois, tels que le calcul des constantes
            intrinsics = cameraIntrinsics(obj.FocalLength, obj.PrincipalPoint, obj.ImageSize);
            obj.VslamObj = monovslam(intrinsics, ...
                                     'ScaleFactor', obj.ScaleFactor, ...
                                     'NumLevels', obj.NumLevels, ...
                                     'MaxNumPoints', obj.MaxNumPoints, ...
                                     'SkipMaxFrames', obj.SkipMaxFrames, ...
                                     'TrackFeatureRange', obj.TrackFeatureRange, ...
                                     'LoopClosureThreshold', obj.LoopClosureThreshold, ...
                                     'Verbose', obj.Verbose, ...
                                     'ThreadLevel', obj.ThreadLevel);
            obj.Pose = eye(4); % Initialisation de la matrice de pose
        end

        function [pose, isTrackingLost, xyzPoints] = stepImpl(obj, I)
            % Implémente l'algorithme. Calcule la pose en fonction de l'image d'entrée I.
            addFrame(obj.VslamObj, I);
            
            if hasNewKeyFrame(obj.VslamObj)
                posesStruct = poses(obj.VslamObj);
                obj.Pose = posesStruct(end).T;
            end
            
            isTrackingLost = ~uint8(checkStatus(obj.VslamObj));
            xyzPoints = mapPoints(obj.VslamObj);
            pose = obj.Pose;
            
            % Assigner les résultats à l'espace de travail de base
            translation_matrix = pose(1:3, 4);
            assignin('base', 'objectPositions', translation_matrix);
            assignin('base', 'slamPoints', xyzPoints);
        end

        function resetImpl(obj)
            % Réinitialise les propriétés discrètes
            obj.Pose = eye(4);
        end

        %% Fonctions de sauvegarde/restauration
        function s = saveObjectImpl(obj)
            % Sauvegarde l'objet
            s = saveObjectImpl@matlab.System(obj);
            s.VslamObj = obj.VslamObj;
            s.Pose = obj.Pose;
        end

        function loadObjectImpl(obj, s, wasLocked)
            % Charge l'objet
            loadObjectImpl@matlab.System(obj, s, wasLocked);
            obj.VslamObj = s.VslamObj;
            obj.Pose = s.Pose;
        end

        %% Fonctions Simulink
        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Retourne la taille pour chaque port de sortie
            out1 = [4, 4];
            out2 = [1, 1];
            out3 = [100000, 3]; % La première dimension est variable, la seconde est fixée à 3
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(obj)
            % Retourne le type de données pour chaque port de sortie
            out1 = 'double';
            out2 = 'logical';
            out3 = 'double';
        end

        function [out1, out2, out3] = isOutputComplexImpl(obj)
            % Retourne false pour chaque port de sortie avec des données non complexes
            out1 = false;
            out2 = false;
            out3 = false;
        end

        function [out1, out2, out3] = isOutputFixedSizeImpl(obj)
            % Retourne true pour chaque port de sortie avec une taille fixe, sauf pour out3
            out1 = true;
            out2 = true;
            out3 = false;
        end

        function name1 = getInputNamesImpl(obj)
            % Retourne le nom du port d'entrée pour le bloc System
            name1 = 'Image';
        end

        function [name1, name2, name3] = getOutputNamesImpl(obj)
            % Retourne les noms des ports de sortie pour le bloc System
            name1 = 'Camera Pose';
            name2 = 'Tracking Lost';
            name3 = 'XYZ Points';
        end
    end
end
