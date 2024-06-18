classdef Yolo_Object_Detector < matlab.System
    % Yolo_Object_Detector - Détecte des objets et calcule leur position en XYZ

    properties (Access = private)
        yolo            % Détecteur YOLO
        focalLength     % Longueur focale de la caméra
        objectDimensions % Tailles moyennes des objets (hauteur, largeur, profondeur)
        targetObject    % Objet cible à détecter
        imageSize       % Taille de l'image [height, width]
        objectXYZrotmatrix % Positions des objets avec matrice de rotation
        objectXYZangleteta % Positions des objets avec angles theta
        distances       % Distances des objets détectés
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
            
            % Charger les dimensions moyennes des objets à partir d'un fichier
            data = load('Matlab_System/objectDimensions.mat');
            obj.objectDimensions = data.objectDimensions;
            
            % Définir l'objet cible
            obj.targetObject = 'tvmonitor'; % Spécifiez ici l'objet cible
            
            % Initialiser les positions et les distances
            obj.objectXYZrotmatrix = [];
            obj.objectXYZangleteta = [];
            obj.distances = [];
        end

        %% Détection des objets et calcul de leurs positions
        function [objectXYZrotmatrix, distances, annotatedImage, objectXYZangleteta] = stepImpl(obj, I, translationMatrix, rotationMatrix)
            % I : image d'entrée
            % translationMatrix : matrice de translation 3x1 du drone
            % rotationMatrix : matrice de rotation 3x3 du drone

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

            % Initialiser les positions des objets et leurs distances
            objectXYZrotmatrix = zeros(size(bboxs, 1), 3);
            objectXYZangleteta = zeros(size(bboxs, 1), 3);
            distances = zeros(size(bboxs, 1), 1);

            % Annoter l'image
            annotatedImage = I;

            % Si aucun objet n'est détecté, retourner l'image annotée sans modification
            if isempty(bboxs)
                return;
            end

            % Paramètres de l'image et de la boîte englobante
            W = size(I, 2); % Largeur de l'image
            H = size(I, 1); % Hauteur de l'image

            % Parcourir chaque boîte englobante détectée
            for i = 1:size(bboxs, 1)
                % Définir les variables pour le calcul des positions
                x = bboxs(i, 1);
                y = bboxs(i, 2);
                w = bboxs(i, 3);
                h = bboxs(i, 4);

                % Calcul des centres
                x_image_center = W / 2;
                y_image_center = H / 2;
                x_center = x + w / 2;
                y_center = y + h / 2;

                % Calcul des angles θx et θy
                % Calcul de l'angle de vision horizontal (alpha)
                alpha = 2 * atan(obj.imageSize(2) / (2 * obj.focalLength(1))) * (180 / pi);

                % Calcul de l'angle de vision vertical (beta)
                beta = 2 * atan(obj.imageSize(1) / (2 * obj.focalLength(2))) * (180 / pi);

                theta_x = ((x_center - x_image_center) / x_image_center) * (alpha / 2);
                theta_y = ((y_center - y_image_center) / y_image_center) * (beta / 2);

                % Récupérer les dimensions moyennes de l'objet
                if isfield(obj.objectDimensions, obj.targetObject)
                    dimensions = obj.objectDimensions.(obj.targetObject);
                    realHeight = dimensions(1);
                    realWidth = dimensions(2);

                    % Calcul de la distance D
                    Dh = (obj.focalLength(1) * realHeight) / h;
                    Dw = (obj.focalLength(1) * realWidth) / w;
                    D = (Dh + Dw) / 2;

                    % Calcul de la position (X, Y, Z) en utilisant les angles θx et θy
                    X = D * tand(theta_x);
                    Y = D * tand(theta_y);
                    Z = D;

                    % Calculer le vecteur de direction dans le référentiel global
                    direction_local = [1; 0; 0]; % Vecteur unitaire dans le référentiel de l'objet
                    direction_global = rotationMatrix * direction_local;

                    % Calculer la position de l'objet en XYZ en utilisant la matrice de rotation
                    objectPosrotmatrix = translationMatrix + D * direction_global;

                    % Calculer la position de l'objet en XYZ en utilisant les angles θx et θy
                    objectPosangleteta = translationMatrix + [X; Y; Z];

                    % Appliquer les transformations de référentiel (sans changement)
                    transformedPosrotmatrix = [objectPosrotmatrix(1); objectPosrotmatrix(2); objectPosrotmatrix(3)];
                    transformedPosangleteta = [objectPosangleteta(1); objectPosangleteta(2); objectPosangleteta(3)];

                    % Stocker la position de l'objet transformée
                    objectXYZrotmatrix(i, :) = transformedPosrotmatrix';
                    objectXYZangleteta(i, :) = transformedPosangleteta';

                    % Stocker la distance
                    distances(i) = D;

                    % Annoter l'image avec la distance et la position
                    labels{i} = sprintf('%s (%.2f m, [%.2f, %.2f, %.2f], [%.2f, %.2f, %.2f])', obj.targetObject, D, transformedPosrotmatrix(1), transformedPosrotmatrix(2), transformedPosrotmatrix(3), transformedPosangleteta(1), transformedPosangleteta(2), transformedPosangleteta(3));
                    annotatedImage = insertObjectAnnotation(I, "rectangle", bboxs(i, :), labels{i}, 'Color', 'yellow');
                end
            end

            % Enregistrer les résultats dans le workspace
            assignin('base', 'objectXYZrotmatrix', obj.objectXYZrotmatrix);
            assignin('base', 'objectXYZangleteta', obj.objectXYZangleteta);
            assignin('base', 'distances', obj.distances);
        end

        %% Définir les tailles des sorties
        function [out1, out2, out3, out4] = getOutputSizeImpl(~)
            % Retourner la taille de chaque port de sortie
            out1 = [1, 3]; % objectXYZrotmatrix
            out2 = [1, 1]; % distance
            out3 = [720, 960, 3]; % annotatedImage (exemple de taille d'image)
            out4 = [1, 3]; % objectXYZangleteta
        end

        %% Définir les types de données des sorties
        function [out1, out2, out3, out4] = getOutputDataTypeImpl(~)
            % Retourner le type de données de chaque port de sortie
            out1 = 'double'; % objectXYZrotmatrix
            out2 = 'double'; % distance
            out3 = 'uint8'; % annotatedImage
            out4 = 'double'; % objectXYZangleteta
        end

        %% Définir si les sorties sont complexes
        function [out1, out2, out3, out4] = isOutputComplexImpl(~)
            % Retourner vrai pour chaque port de sortie avec des données complexes
            out1 = false; % objectXYZrotmatrix
            out2 = false; % distance
            out3 = false; % annotatedImage
            out4 = false; % objectXYZangleteta
        end

        %% Définir si les tailles des sorties sont fixes
        function [out1, out2, out3, out4] = isOutputFixedSizeImpl(~)
            % Retourner vrai pour chaque port de sortie avec une taille fixe
            out1 = true; % objectXYZrotmatrix
            out2 = true; % distance
            out3 = true; % annotatedImage
            out4 = true; % objectXYZangleteta
        end

        %% Définir les noms des ports d'entrée
        function [name1, name2, name3] = getInputNamesImpl(~)
            % Retourner les noms des ports d'entrée pour le bloc système
            name1 = 'Image';
            name2 = 'TranslationMatrix';
            name3 = 'RotationMatrix';
        end

        %% Définir les noms des ports de sortie
        function [name1, name2, name3, name4] = getOutputNamesImpl(~)
            % Retourner les noms des ports de sortie pour le bloc système
            name1 = 'objectXYZrotmatrix';
            name2 = 'Distance';
            name3 = 'AnnotatedImage';
            name4 = 'objectXYZangleteta';
        end
    end
end
