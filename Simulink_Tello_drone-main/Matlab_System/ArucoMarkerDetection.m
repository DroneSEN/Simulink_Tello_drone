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
        
        function [output_image, camera_pos_ref_aruco, euler_angles, detection] = stepImpl(obj, I)
            % I : image d'entrée

            % Correction de la rotation et mirroir
            I = flip(permute(I, [2, 1, 3]), 2);

            % Correction de la distorsion de l'image
            I = undistortImage(I, obj.intrinsics);

            % Estimation des poses des marqueurs
            [ids, locs, poses] = readArucoMarker(I, obj.markerFamily, obj.intrinsics, obj.markerSizeInMM);

            % Initialisation des variables de sortie
            output_image = I;

            cameraPositions = NaN(obj.maxMarkers, 3); % Positions de la caméra
            euler_angles = NaN(obj.maxMarkers, 3); % Angles d'Euler
            
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
                output_image = insertShape(output_image, "Line", axesPoints, ...
                    'Color', ["red","green","blue"], 'LineWidth', 10);
                
                % Calcul de la position de la caméra par rapport au marqueur
                cameraPositions(i, :) = -poses(i).Rotation' * poses(i).Translation';
                
                % Calcul des angles d'Euler à partir de la matrice de rotation
                euler_angles(i, :) = rotm2eul(poses(i).Rotation, 'ZYX');
            end
            
            % Conversion des positions de la caméra en mètres
            cameraPositions_metre_XYZ = cameraPositions / 1000; % Conversion de mm en mètres
            
            % Conversion des positions au repère optitrack
            camera_pos_ref_aruco = cameraPositions_metre_XYZ(1,:);

            % Remplir les emplacements non utilisés avec des zéros
            for i = (length(poses)+1):obj.maxMarkers
                camera_pos_ref_aruco(i, :) = [0, 0, 0];
                euler_angles(i, :) = [0, 0, 0];
            end

            output_image = uint8(output_image);

            % Validation des données
            if length(poses) >= 1
                % Si on a une position estimé, on enable le flag
                detection = 1;

                % On actualise les valeurs précédentes
                obj.previousValidPos = camera_pos_ref_aruco;
                obj.previousValidEulerAngles = euler_angles;
            else
                % Sinon, on désactive le flag
                detection = 0;

                % On retourne les valeurs précédentes
                camera_pos_ref_aruco = obj.previousValidPos;
                euler_angles = obj.previousValidEulerAngles;
            end
        end
        
        function resetImpl(obj)
            % Initialiser / réinitialiser les propriétés discrètes
        end
        
        function [output_image, camera_pos_ref_aruco, euler_angles, detection] = getOutputSizeImpl(obj)
            % Retourner la taille de chaque port de sortie
            output_image = [ obj.imageSize(2), obj.imageSize(1), 3];
            camera_pos_ref_aruco = [obj.maxMarkers, 3];  % Taille fixe pour les positions
            euler_angles = [obj.maxMarkers, 3];  % Taille fixe pour les angles d'Euler
            detection = [1,1];
        end
        
        function [output_image, camera_pos_ref_aruco, euler_angles, detection] = getOutputDataTypeImpl(~)
            % Retourner le type de données de chaque port de sortie
            output_image = 'uint8';
            camera_pos_ref_aruco = 'double';
            euler_angles = 'double';
            detection = 'double';
        end
        
        function [output_image, camera_pos_ref_aruco, euler_angles, detection] = isOutputComplexImpl(~)
            % Retourner vrai pour chaque port de sortie avec des données complexes
            output_image = false;
            camera_pos_ref_aruco = false;
            euler_angles = false;
            detection = false;
        end
        
        function [output_image, camera_pos_ref_aruco, euler_angles, detection] = isOutputFixedSizeImpl(~)
            % Retourner vrai pour chaque port de sortie avec une taille fixe
            output_image = true;
            camera_pos_ref_aruco = true;  % Taille fixe pour les positions
            euler_angles = true;  % Taille fixe pour les angles d'Euler
            detection = true;
        end
    end
end
