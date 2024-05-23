classdef Yolo_Object_Detector < matlab.System
    % Yolo_Object_Detector - Détecte des objets et calcule leur position en XYZ
    properties (Access = private)
        yolo % Détecteur YOLO
        focalLength % Longueur focale de la caméra
        objectDimensions % Tailles moyennes des objets (hauteur, largeur, profondeur)
    end

    methods (Access = protected)
        %% Initialisation
        function setupImpl(obj)
            name = 'tiny-yolov4-coco';
            % Initialiser YOLO
            obj.yolo = yolov4ObjectDetector(name);
            obj.focalLength = [903, 906]; % Exemple de focal length

            % Charger les dimensions moyennes des objets à partir du fichier
            data = load('Matlab_System/objectDimensions.mat');
            obj.objectDimensions = data.objectDimensions;
        end

        %% Détection des objets et calcul de leurs positions
        function [objectXYZ, distances, annotatedImage] = stepImpl(obj, I, translationMatrix, rotationMatrix)
            % I : image d'entrée
            % translationMatrix : matrice de translation 3x1 du drone
            % rotationMatrix : matrice de rotation 3x3 du drone

            % Détecter les objets
            [bboxs, ~, labels] = detect(obj.yolo, I, 'Threshold', 0.1);

            % Convertir les labels en tableau de cellules
            if ~iscell(labels)
                labels = cellstr(labels);
            end

            % Initialiser les positions des objets et leurs dimensions
            objectXYZ = zeros(100, 3);
            distances = zeros(100, 1); % Tableau pour stocker les distances
            objectColors = repmat("yellow", size(bboxs, 1), 1);

            % Taille de l'image d'entrée
            %[imgHeight, imgWidth, ~] = size(I);

            % Calculer la distance pour chaque objet détecté et ajouter l'annotation
            for i = 1:size(bboxs, 1)
                % Récupérer la hauteur et la largeur de la boîte englobante
                bboxHeight = bboxs(i, 4);
                bboxWidth = bboxs(i, 3);

                if isfield(obj.objectDimensions, labels{i})
                    % Récupérer les dimensions moyennes de l'objet
                    dimensions = obj.objectDimensions.(labels{i});
                    realHeight = dimensions(1);
                    realWidth = dimensions(2);
                    % realDepth = dimensions(3);

                    % Calculer la distance en utilisant la largeur de la boîte
                    distanceW = (realWidth * obj.focalLength(1)) / bboxWidth;
                    distanceH = (realHeight * obj.focalLength(1)) / bboxHeight;
                    distance = (distanceW + distanceH) / 2;


                    distances(i) = distance; % Stocker la distance

                    % Calculer le vecteur de direction
                    direction_local = [1; 0; 0]; % Vecteur unitaire dans le référentiel de l'objet
                    direction_global = rotationMatrix * direction_local;
                    
                    % Calculer la position de l'objet
                    objectPos = translationMatrix + distance * direction_global;

                    % Appliquer les transformations de référentiel
                    transformedPos = [-objectPos(1); objectPos(2); -objectPos(3)];

                    % Stocker la position de l'objet transformée
                    objectXYZ(i, :) = transformedPos';

                    % Annoter l'image
                    labels{i} = sprintf('%s (%.2f m, [%.2f, %.2f, %.2f])', labels{i}, distance, transformedPos(1), transformedPos(2), transformedPos(3));

                    % Déterminer la couleur en fonction de la distance
                    if distance > 3
                        objectColors(i) = 'green';
                    elseif distance > 1
                        objectColors(i) = 'yellow';
                    else
                        objectColors(i) = 'red';
                    end
                end
            end

            % Annoter l'image avec les boîtes et les distances
            annotatedImage = insertObjectAnnotation(I, "rectangle", bboxs, labels, 'Color', cellstr(objectColors));

            % Supprimer les lignes non utilisées des objets détectés
            nonZeroIdx = any(objectXYZ ~= 0, 2);
            objectXYZ = objectXYZ(nonZeroIdx, :);
            distances = distances(nonZeroIdx, :); % Supprimer les lignes non utilisées

            % Remplir les variables avec des zéros pour assurer une taille fixe de [100, 3] pour objectXYZ et [100, 1] pour distances
            objectXYZ = [objectXYZ; zeros(100 - size(objectXYZ, 1), 3)];
            distances = [distances; zeros(100 - size(distances, 1), 1)];
        end

        %% Définir les tailles des sorties
        function [out1, out2, out3] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [100, 3]; % objectXYZ
            out2 = [100, 1]; % distance
            out3 = [720, 960, 3]; % annotatedImage
        end

        %% Définir les types de données des sorties
        function [out1, out2, out3] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = 'double'; % objectXYZ
            out2 = 'double'; % distance
            out3 = 'uint8'; % annotatedImage
        end

        %% Définir si les sorties sont complexes
        function [out1, out2, out3] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false; % objectXYZ
            out2 = false; % distance
            out3 = false; % annotatedImage
        end

        %% Définir si les tailles des sorties sont fixes
        function [out1, out2, out3] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = true; % objectXYZ
            out2 = true; % distance
            out3 = true; % annotatedImage
        end

        %% Définir les noms des ports d'entrée
        function [name1, name2, name3] = getInputNamesImpl(~)
            % Return input port names for System block
            name1 = 'Image';
            name2 = 'TranslationMatrix';
            name3 = 'RotationMatrix';
        end

        %% Définir les noms des ports de sortie
        function [name1, name2, name3] = getOutputNamesImpl(~)
            % Return output port names for System block
            name1 = 'ObjectXYZ';
            name2 = 'Distance';
            name3 = 'AnnotatedImage';
        end
    end
end
