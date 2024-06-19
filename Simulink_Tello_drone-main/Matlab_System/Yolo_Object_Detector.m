classdef Yolo_Object_Detector < matlab.System
    % Yolo_Object_Detector - Détecte des objets et calcule leur position en XYZ

    properties (Access = private)
        yolo                % Détecteur YOLO
        focalLength         % Longueur focale de la caméra
        objectDimensions    % Tailles moyennes des objets (hauteur, largeur, profondeur)
        targetObject        % Objet cible à détecter
        imageSize           % Taille de l'image [height, width]
        imagecenterpoint    % Point central de l'image
        lastKnownPos        % Dernière position connue de l'objet
        intrinsicMatrix     % Matrice intrinsèque (matrice de projection)
    end

    methods (Access = protected)
        %% Initialisation
        function setupImpl(obj)
            % Initialiser le détecteur YOLO
            name = 'tiny-yolov4-coco';
            obj.yolo = yolov4ObjectDetector(name);
            
            % Définir la longueur focale de la caméra et le point principal (exemples de valeurs)
            obj.focalLength = [878.5236, 883.0086];
            obj.imagecenterpoint = [488.4168, 363.6072];
            
            % Définir la taille de l'image [height, width] en pixels
            obj.imageSize = [720, 960];
            
            % Charger les dimensions moyennes des objets à partir d'un fichier
            data = load('Matlab_System/objectDimensions.mat');
            obj.objectDimensions = data.objectDimensions;
            
            % Définir l'objet cible (initialisation)
            obj.targetObject = 'tvmonitor'; 
            
            % Initialiser la dernière position connue
            obj.lastKnownPos = [NaN; NaN; NaN];
            
            % Définir la matrice intrinsèque (de projection) basée sur la longueur focale et le centre de l'image
            obj.intrinsicMatrix = [obj.focalLength(1), 0, obj.imagecenterpoint(1);
                                   0, obj.focalLength(2), obj.imagecenterpoint(2);
                                   0, 0, 1];
        end

        %% Détection des objets et calcul de leurs positions
        function [objectPos, objectType, annotatedImage] = stepImpl(obj, I)
            % I : image d'entrée

            % Détecter les objets dans l'image
            [bboxs, ~, labels] = detect(obj.yolo, I, 'Threshold', 0.1);

            % Convertir les labels en tableau de cellules si nécessaire
            if ~iscell(labels)
                labels = cellstr(labels);
            end

            % Filtrer les détections pour ne garder que l'objet cible
            targetIdx = strcmp(labels, obj.targetObject);
            bboxs = bboxs(targetIdx, :);
            labels = labels(targetIdx);

            % Initialiser la position de l'objet et le type d'objet
            objectType = "";
            annotatedImage = I;

            % Si aucun objet n'est détecté, retourner la dernière position connue
            if isempty(bboxs)
                objectPos = obj.lastKnownPos;
                return;
            end

            % Si plusieurs objets sont détectés, retourner la dernière position connue
            if size(bboxs, 1) > 1
                objectPos = obj.lastKnownPos;
                return;
            end

            % Paramètres de l'image et de la boîte englobante
            W = obj.imageSize(2); % Largeur de l'image
            H = obj.imageSize(1); % Hauteur de l'image

            % Définir les variables pour le calcul des positions
            x = bboxs(1, 1);
            y = bboxs(1, 2);
            w = bboxs(1, 3);
            h = bboxs(1, 4);

            % Calcul des centres de la boîte englobante
            x_center = x + w / 2;
            y_center = y + h / 2;

            % Récupérer les dimensions moyennes de l'objet
            if isfield(obj.objectDimensions, obj.targetObject)
                dimensions = obj.objectDimensions.(obj.targetObject);
                realHeight = dimensions(1);
                realWidth = dimensions(2);
                realDepth = dimensions(3);

                % Calcul de la distance D
                Dh = (obj.focalLength(1) * realHeight) / h;
                Dw = (obj.focalLength(1) * realWidth) / w;
                D = (Dh + Dw) / 2;

                % Ajuster la distance avec la moitié de la profondeur de l'objet
                D = D + (realDepth / 2);

                % Calcul des coordonnées dans le référentiel de la caméra
                % Utilisation de la matrice intrinsèque pour convertir les coordonnées de pixels en mètres
                invIntrinsic = inv(obj.intrinsicMatrix);
                pixel_coords = [x_center; y_center; 1];
                camera_coords = invIntrinsic * (D * pixel_coords);

                X_cam = camera_coords(1);
                Y_cam = camera_coords(2);
                Z_cam = D;

                % Conversion des coordonnées dans le référentiel de la caméra vers celui du drone
                % Référentiel caméra : X vers la droite, Y vers le haut, Z vers l'avant
                % Référentiel drone : X vers l'avant, Y vers la droite, Z vers le bas
                % Correspondance : X_drone = Z_cam, Y_drone = X_cam, Z_drone = -Y_cam
                objectPos_cam_torefdrone = [Z_cam; X_cam; -Y_cam];
                
                % Transformation en coordonnées du drone
                objectPos = objectPos_cam_torefdrone;
                objectPos = double(objectPos);  % S'assurer que objectPos est de type double

                % Annoter l'image avec la distance et la position
                label = sprintf('%s (%.2f m, [%.2f, %.2f, %.2f])', obj.targetObject, D, objectPos(1), objectPos(2), objectPos(3));
                annotatedImage = insertObjectAnnotation(I, "rectangle", bboxs(1, :), label, 'Color', 'yellow');
                
                % Définir le type d'objet
                objectType = string(obj.targetObject);  % S'assurer que objectType est de type string
                
                % Mettre à jour la dernière position connue
                obj.lastKnownPos = objectPos;
            end
        end

        %% Définir les tailles des sorties
        function [out1, out2, out3] = getOutputSizeImpl(~)
            % Retourner la taille de chaque port de sortie
            out1 = [3, 1]; % objectPos
            out2 = [1, 1]; % objectType
            out3 = [720, 960, 3]; % annotatedImage (exemple de taille d'image)
        end

        %% Définir les types de données des sorties
        function [out1, out2, out3] = getOutputDataTypeImpl(~)
            % Retourner le type de données de chaque port de sortie
            out1 = 'double'; % objectPos
            out2 = 'string'; % objectType
            out3 = 'uint8'; % annotatedImage
        end

        %% Définir si les sorties sont complexes
        function [out1, out2, out3] = isOutputComplexImpl(~)
            % Retourner vrai pour chaque port de sortie avec des données complexes
            out1 = false; % objectPos
            out2 = false; % objectType
            out3 = false; % annotatedImage
        end

        %% Définir si les tailles des sorties sont fixes
        function [out1, out2, out3] = isOutputFixedSizeImpl(~)
            % Retourner vrai pour chaque port de sortie avec une taille fixe
            out1 = true; % objectPos
            out2 = true; % objectType
            out3 = true; % annotatedImage
        end

        %% Définir les noms des ports d'entrée
        function [name1] = getInputNamesImpl(~)
            % Retourner les noms des ports d'entrée pour le bloc système
            name1 = 'Image';
        end

        %% Définir les noms des ports de sortie
        function [name1, name2, name3] = getOutputNamesImpl(~)
            % Retourner les noms des ports de sortie pour le bloc système
            name1 = 'ObjectPos';
            name2 = 'ObjectType';
            name3 = 'AnnotatedImage';
        end
    end
end
