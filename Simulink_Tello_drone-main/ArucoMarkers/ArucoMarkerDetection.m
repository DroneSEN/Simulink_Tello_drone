classdef ArucoMarkerDetection < matlab.System & matlab.system.mixin.Propagates
    % ArucoMarkerDetection - Détecte des marqueurs ArUco et calcule la position et l'orientation de la caméra
    
    properties
        % Propriétés publiques et ajustables
        markerSizeInMM = 150; % Taille des marqueurs ArUco en millimètres
        markerFamily = 'DICT_4X4_250'; % Famille de marqueurs ArUco
        maxMarkers = 1; % Nombre maximum de marqueurs détectables
        
        focalLength; % Longueur focale de la caméra
        principalPoint; % Point principal de la caméra
        imageSize; % Taille de l'image
    end
    
    properties(Access = private)
        % Constantes pré-calculées
        intrinsics;  % Objet des paramètres intrinsèques de la caméra
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Configuration des paramètres intrinsèques de la caméra avec les propriétés fournies
            obj.intrinsics = cameraIntrinsics(obj.focalLength, obj.principalPoint, obj.imageSize);
        end
        
        function [outputImage, cameraPositions_repere_optitrack, eulerAngles] = stepImpl(obj, I)
            % I : image d'entrée
            
            % Correction de la distorsion de l'image
            I = undistortImage(I, obj.intrinsics);
            
            % Estimation des poses des marqueurs
            [ids, locs, poses] = readArucoMarker(I, obj.markerFamily, obj.intrinsics, obj.markerSizeInMM);
            
            % Initialisation des variables de sortie
            % outputImage = I;
            outputImage = zeros(obj.imageSize(1), obj.imageSize(2), 3); % Image annotée en sortie
            outputImage(:,:,1) = I;
            outputImage(:,:,2) = I;
            outputImage(:,:,3) = I;

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
                imagePoints = worldToImage(obj.intrinsics, poses(i).Rotation, poses(i).Translation, worldPoints);
                
                % Points pour les axes
                axesPoints = [imagePoints(1,:) imagePoints(2,:);
                              imagePoints(1,:) imagePoints(3,:);
                              imagePoints(1,:) imagePoints(4,:)];                
                
                % Dessin des axes colorés sur l'image
                outputImage = insertShape(outputImage, "Line", axesPoints, ...
                    'Color', ["red","green","blue"], 'LineWidth', 10);
                
                % Calcul de la position de la caméra par rapport au marqueur
                cameraPositions(i, :) = -poses(i).Translation * poses(i).Rotation';
                
                % Calcul des angles d'Euler à partir de la matrice de rotation
                eulerAngles(i, :) = rotm2eul(poses(i).Rotation, 'ZYX');
            end
            
            % Conversion des positions de la caméra en mètres
            cameraPositions_metre_XYZ = cameraPositions / 1000; % Conversion de mm en mètres
            
            % Conversion des positions au repère optitrack
            cameraPositions_repere_optitrack = [cameraPositions_metre_XYZ(:,1), ...
                                                cameraPositions_metre_XYZ(:,3), ...
                                                -cameraPositions_metre_XYZ(:,2)];

            % Remplir les emplacements non utilisés avec des zéros
            for i = (length(poses)+1):obj.maxMarkers
                cameraPositions_repere_optitrack(i, :) = [0, 0, 0];
                eulerAngles(i, :) = [0, 0, 0];
            end

            outputImage = uint8(outputImage);
        end
        
        function resetImpl(obj)
            % Initialiser / réinitialiser les propriétés discrètes
        end
        
        function [outputImage, cameraPositions_repere_optitrack, eulerAngles] = getOutputSizeImpl(obj)
            % Retourner la taille de chaque port de sortie
            outputImage = [obj.imageSize(1), obj.imageSize(2), 3];
            cameraPositions_repere_optitrack = [obj.maxMarkers, 3];  % Taille fixe pour les positions
            eulerAngles = [obj.maxMarkers, 3];  % Taille fixe pour les angles d'Euler
        end
        
        function [outputImage, cameraPositions_repere_optitrack, eulerAngles] = getOutputDataTypeImpl(~)
            % Retourner le type de données de chaque port de sortie
            outputImage = 'uint8';
            cameraPositions_repere_optitrack = 'double';
            eulerAngles = 'double';
        end
        
        function [outputImage, cameraPositions_repere_optitrack, eulerAngles] = isOutputComplexImpl(~)
            % Retourner vrai pour chaque port de sortie avec des données complexes
            outputImage = false;
            cameraPositions_repere_optitrack = false;
            eulerAngles = false;
        end
        
        function [outputImage, cameraPositions_repere_optitrack, eulerAngles] = isOutputFixedSizeImpl(~)
            % Retourner vrai pour chaque port de sortie avec une taille fixe
            outputImage = true;
            cameraPositions_repere_optitrack = true;  % Taille fixe pour les positions
            eulerAngles = true;  % Taille fixe pour les angles d'Euler
        end
    end
end
