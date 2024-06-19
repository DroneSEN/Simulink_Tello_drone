classdef Yolo_Object_Detector < matlab.System
    % Yolo_Object_Detector - Détecte des objets et calcule leur position en XYZ

    properties (Access = private)
        yolo            % Détecteur YOLO
        focalLength     % Longueur focale de la caméra
        objectDimensions % Tailles moyennes des objets (hauteur, largeur, profondeur)
        targetObject    % Objet cible à détecter
        imageSize       % Taille de l'image [height, width]
        imagecenterpoint
    end

    methods (Access = protected)
        %% Initialisation
        function setupImpl(obj)
            % Définir le nom du modèle YOLO utilisé
            name = 'tiny-yolov4-coco';
            
            % Initialiser le détecteur YOLO
            obj.yolo = yolov4ObjectDetector(name);
            
            % Définir la longueur focale de la caméra (exemple de valeurs)
            obj.focalLength = [878.5, 883]; % Exemples de valeurs
            
            % Définir la taille de l'image [height, width] en pixels
            obj.imageSize = [720, 960];
            obj.imagecenterpoint = [488.416,363.607];
            % Charger les dimensions moyennes des objets à partir d'un fichier
            data = load('Matlab_System/objectDimensions.mat');
            obj.objectDimensions = data.objectDimensions;
            
            % Définir l'objet cible (initialisation)
            obj.targetObject = 'tvmonitor'; % Spécifiez ici l'objet cible initial
        end

        %% Détection des objets et calcul de leurs positions
        function [objectPos, objectType, annotatedImage] = stepImpl(obj, I, rotationMatrix, translationMatrix)
            % I : image d'entrée
            % rotationMatrix : matrice de rotation 3x3 du drone
            % translationMatrix : vecteur de translation 3x1 du drone

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
            objectPos = [NaN; NaN; NaN];
            objectType = '';

            % Annoter l'image
            annotatedImage = I;

            % Si aucun objet n'est détecté ou plusieurs objets sont détectés, retourner NaN et type vide
            if isempty(bboxs) || size(bboxs, 1) > 1
                return;
            end

            % Paramètres de l'image et de la boîte englobante
            W = size(I, 2); % Largeur de l'image
            H = size(I, 1); % Hauteur de l'image

            % Définir les variables pour le calcul des positions
            x = bboxs(1, 1);
            y = bboxs(1, 2);
            w = bboxs(1, 3);
            h = bboxs(1, 4);

            % Calcul des centres
            x_image_center = W / 2;
            y_image_center = H / 2;
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

                % Transformation des coordonnées en utilisant la matrice de rotation
                X = D;
                Y = D * (x_center - x_image_center) / obj.focalLength(1);
                Z = D * (y_center - y_image_center) / obj.focalLength(2);
                
                % Coordonnées dans le référentiel de la caméra
                objectPos_cam = [X; Y; Z];
                
                % Transformation en coordonnées du drone
                objectPos = rotationMatrix * objectPos_cam + translationMatrix;

                % Annoter l'image avec la distance et la position
                label = sprintf('%s (%.2f m, [%.2f, %.2f, %.2f])', obj.targetObject, D, objectPos(1), objectPos(2), objectPos(3));
                annotatedImage = insertObjectAnnotation(I, "rectangle", bboxs(1, :), label, 'Color', 'yellow');
                
                % Définir le type d'objet
                objectType = obj.targetObject;
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
            out2 = 'char'; % objectType
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
        function [name1, name2, name3] = getInputNamesImpl(~)
            % Retourner les noms des ports d'entrée pour le bloc système
            name1 = 'Image';
            name2 = 'RotationMatrix';
            name3 = 'TranslationMatrix';
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
