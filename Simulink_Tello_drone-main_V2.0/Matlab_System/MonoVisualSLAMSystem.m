classdef MonoVisualSLAMSystem < matlab.System
    % MonoVisualSLAMSystem Algorithme de SLAM visuel monoculaire
    %
    % Cette classe est un exemple de fonction d'aide et peut être modifiée
    % ou supprimée dans les futures versions.

    % Propriétés publiques, non modifiables
    properties(Nontunable)
        % FocalLength Longueur focale de la caméra
        FocalLength = [366.7411, 362.9497];
        % PrincipalPoint Centre principal de la caméra
        PrincipalPoint = [252.67, 206.61];
        % ImageSize Taille de l'image
        ImageSize = [360, 480];
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
            numPoints = 3000;
            numSkipFrames = 5;
            obj.VslamObj = monovslam(intrinsics, MaxNumPoints=numPoints, SkipMaxFrames=numSkipFrames);
            obj.Pose = eye(4); % Initialisation de la matrice de pose
        end

        function [pose, isTrackingLost, xyzPoints] = stepImpl(obj, I)
            % Implémente l'algorithme. Calcule la pose en fonction de l'image d'entrée I.
            addFrame(obj.VslamObj, I);
            while ~isDone(obj.VslamObj)
                if hasNewKeyFrame(obj.VslamObj)
                    [camPoses, ~] = poses(obj.VslamObj);
                    p = camPoses(end);
                    obj.Pose = p.A;
                end
            end
            pose = obj.Pose;
            isTrackingLost = ~uint8(checkStatus(obj.VslamObj));
            xyzPoints = mapPoints(obj.VslamObj);
        end

        function resetImpl(obj)
            % Réinitialise les propriétés discrètes
            obj.Pose = eye(4);
        end

        %% Fonctions de sauvegarde/restauration
        function s = saveObjectImpl(obj)
            % Sauvegarde l'objet
            s = saveObjectImpl@matlab.System(obj);
        end

        function loadObjectImpl(obj, s, wasLocked)
            % Charge l'objet
            loadObjectImpl@matlab.System(obj, s, wasLocked);
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
            out2 = 'boolean';
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

    methods(Static, Access = protected)
        %% Fonctions de personnalisation Simulink
        % function header = getHeaderImpl
        %     % Définir le panneau d'en-tête pour la boîte de dialogue du bloc System
        %     header = matlab.system.display.Header(mfilename("class"));
        % end

        % function group = getPropertyGroupsImpl
        %     % Définir la ou les sections de propriétés pour la boîte de dialogue du bloc System
        %     group = matlab.system.display.Section(mfilename("class"));
        % end

        % function flag = showSimulateUsingImpl
        %     % Retourne false si le mode de simulation est masqué dans la boîte de dialogue du bloc System
        %     flag = true;
        % end
    end
end
