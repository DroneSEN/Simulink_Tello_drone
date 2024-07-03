classdef ArucoMarkerDetection < matlab.System & matlab.system.mixin.Propagates
    % ArucoMarkerDetection - Détecte des marqueurs ArUco et calcule la position et l'orientation de la caméra
    
    properties
        % Propriétés publiques et ajustables
        markerSizeInMM = 150;           % Taille des marqueurs ArUco en millimètres
        markerFamily = "DICT_4X4_250";  % Famille de marqueurs ArUco
        maxMarkers = 1;                 % Nombre maximum de marqueurs détectables
        
        focalLength;                    % Longueur focale de la caméra
        principalPoint;                 % Point principal de la caméra
        imageSize;                      % Taille de l'image
    end
    
    properties(Access = private)
        % Constantes pré-calculées
        intrinsics;  % Objet des paramètres intrinsèques de la caméra

        % Valeur valide précédente
        previousValidPos;
        previousValidEulerAngles;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Configuration des paramètres intrinsèques de la caméra avec les propriétés fournies
            load("camera_calibration\Camera_calibration_mat\export_9BA0DC_down.mat", "cameraParams_9BA0DC");
            fprintf("Chargement des données intrinsèques du drone 9BA0DC\n");
            obj.intrinsics = cameraParams_9BA0DC.Intrinsics;

            obj.previousValidPos = zeros(obj.maxMarkers, 3);
            obj.previousValidEulerAngles = zeros(obj.maxMarkers, 3);    
        end
        
        function [outputImage, camera_pos_ref_aruco, eulerAngles, detection] = stepImpl(obj, I)
            % I : image d'entrée

            % Correction de la rotation et mirroir
            I = flip(permute(I, [2, 1, 3]), 2);

            % Correction de la distorsion de l'image
            I = undistortImage(I, obj.intrinsics);

            % Estimation des poses des marqueurs
            [ids, locs, poses] = readArucoMarker(I, obj.markerFamily, obj.intrinsics, obj.markerSizeInMM);

            % Initialisation des variables de sortie
            outputImage = I;

            cameraPositions = NaN(obj.maxMarkers, 3); % Positions de la caméra
            eulerAngles = NaN(obj.maxMarkers, 3); % Angles d'Euler
            
            % Points pour le système de coordonnées de l'objet
            worldPoints = [0 0 0; obj.markerSizeInMM/2 0 0; 0 obj.markerSizeInMM/2 0; 0 0 obj.markerSizeInMM/2];

            % Parcours de chaque pose détectée
            for i = 1:length(poses)
                if i > obj.maxMarkers
                    break;
                end
                
                % Calcul des coordonnées image pour les axes
                % 1. Calcul de la matrice de transformation 3D
                % 2. Projection des points
                tform = rigidtform3d(poses(i).Rotation', poses(i).Translation);
                imagePoints = worldToImage(obj.intrinsics, tform, worldPoints);

                % Points pour les axes
                axesPoints = [imagePoints(1,:) imagePoints(2,:);
                              imagePoints(1,:) imagePoints(3,:);
                              imagePoints(1,:) imagePoints(4,:)];
                
                % Dessin des axes colorés sur l'image
                outputImage = insertShape(outputImage, "Line", axesPoints, ...
                    'Color', ["red","green","blue"], 'LineWidth', 10);
                
                % Calcul de la position de la caméra par rapport au marqueur
                cameraPositions(i, :) = -poses(i).Rotation' * poses(i).Translation';
                
                % Calcul des angles d'Euler à partir de la matrice de rotation
                eulerAngles(i, :) = rotm2eul(poses(i).Rotation, 'ZYX');
            end
            
            % Conversion des positions de la caméra en mètres
            cameraPositions_metre_XYZ = cameraPositions / 1000; % Conversion de mm en mètres
            
            % Conversion des positions au repère optitrack
            camera_pos_ref_aruco = cameraPositions_metre_XYZ(1,:);

            % Remplir les emplacements non utilisés avec des zéros
            for i = (length(poses)+1):obj.maxMarkers
                camera_pos_ref_aruco(i, :) = [0, 0, 0];
                eulerAngles(i, :) = [0, 0, 0];
            end

            outputImage = uint8(outputImage);

            % Validation des données
            if length(poses) >= 1
                % Si on a une position estimé, on enable le flag
                detection = 1;

                % On actualise les valeurs précédentes
                obj.previousValidPos = camera_pos_ref_aruco;
                obj.previousValidEulerAngles = eulerAngles;
            else
                % Sinon, on désactive le flag
                detection = 0;

                % On retourne les valeurs précédentes
                camera_pos_ref_aruco = obj.previousValidPos;
                eulerAngles = obj.previousValidEulerAngles;
            end
        end
        
        function resetImpl(obj)
            % Initialiser / réinitialiser les propriétés discrètes
        end
        
        function [outputImage, camera_pos_ref_aruco, eulerAngles, detection] = getOutputSizeImpl(obj)
            % Retourner la taille de chaque port de sortie
            outputImage = [ obj.imageSize(2), obj.imageSize(1), 3];
            camera_pos_ref_aruco = [obj.maxMarkers, 3];  % Taille fixe pour les positions
            eulerAngles = [obj.maxMarkers, 3];  % Taille fixe pour les angles d'Euler
            detection = [1,1];
        end
        
        function [outputImage, camera_pos_ref_aruco, eulerAngles, detection] = getOutputDataTypeImpl(~)
            % Retourner le type de données de chaque port de sortie
            outputImage = 'uint8';
            camera_pos_ref_aruco = 'double';
            eulerAngles = 'double';
            detection = 'double';
        end
        
        function [outputImage, camera_pos_ref_aruco, eulerAngles, detection] = isOutputComplexImpl(~)
            % Retourner vrai pour chaque port de sortie avec des données complexes
            outputImage = false;
            camera_pos_ref_aruco = false;
            eulerAngles = false;
            detection = false;
        end
        
        function [outputImage, camera_pos_ref_aruco, eulerAngles, detection] = isOutputFixedSizeImpl(~)
            % Retourner vrai pour chaque port de sortie avec une taille fixe
            outputImage = true;
            camera_pos_ref_aruco = true;  % Taille fixe pour les positions
            eulerAngles = true;  % Taille fixe pour les angles d'Euler
            detection = true;
        end
    end
end
