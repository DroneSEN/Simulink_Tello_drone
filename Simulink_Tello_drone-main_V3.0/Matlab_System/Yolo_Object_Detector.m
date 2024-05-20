classdef Yolo_Object_Detector < matlab.System
    % Yolo_Object_Detector - Détecte des objets et calcule leur position en XYZ
    properties
        Offsetpersistent = 1.5
    end
    properties (Access = private)
        yolo % Détecteur YOLO
        focalLength % Longueur focale de la caméra
        objectDimensions % Tailles moyennes des objets (hauteur, largeur, profondeur)
        persistentObjectXYZ % Positions persistantes des objets
        persistentObjectDims % Dimensions persistantes des objets
        imageWithAnnotations % Image avec annotations
        
    end

    methods (Access = protected)
        %% Initialisation
        function setupImpl(obj)
            name = 'tiny-yolov4-coco';
            % Initialiser YOLO
            obj.yolo = yolov4ObjectDetector(name);
            obj.focalLength = [1002.777899955502, 1006.322192826368]; % Exemple de focal length

            % Charger les dimensions moyennes des objets à partir du fichier
            data = load('Matlab_System/objectDimensions.mat');
            obj.objectDimensions = data.objectDimensions;

            % Initialiser les listes persistantes
            obj.persistentObjectXYZ = zeros(100, 3); % Limite à 100 objets persistants
            obj.persistentObjectDims = zeros(100, 3); % Limite à 100 objets persistants
        end

        %% Détection des objets et calcul de leurs positions
        function [objectXYZ, objectDims, persistentObjectXYZ, persistentObjectDims, annotatedImage] = stepImpl(obj, I, translationMatrix, rotationMatrix)
            % I : image d'entrée
            % translationMatrix : matrice de translation 3x1 du drone
            % rotationMatrix : matrice de rotation 3x3 du drone

            % Détecter les objets
            [bboxs, ~, labels] = detect(obj.yolo, I, 'Threshold', 0.8);

            % Convertir les labels en tableau de cellules
            if ~iscell(labels)
                labels = cellstr(labels);
            end

            % Initialiser les positions des objets et leurs dimensions
            objectXYZ = zeros(100, 3);
            objectDims = zeros(100, 3);
            objectColors = repmat("yellow", size(bboxs, 1), 1);

            % Taille de l'image d'entrée
            [imgHeight, imgWidth, ~] = size(I);

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
                    realDepth = dimensions(3);

                    % Calculer la distance en utilisant la largeur de la boîte
                    distance = (realWidth * obj.focalLength(1)) / bboxWidth;

                    % Calculer la position de l'objet dans l'espace (X: profondeur, Y: largeur, Z: hauteur)
                    centerX = bboxs(i, 1) + bboxWidth / 2;
                    centerY = bboxs(i, 2) + bboxHeight / 2;
                    normX = (centerX - imgWidth / 2) / imgWidth;
                    normY = (centerY - imgHeight / 2) / imgHeight;

                    objX = distance; % Position X calculée
                    objY = normX * bboxWidth * distance / obj.focalLength(1); % Position Y calculée en fonction de la largeur de l'image
                    objZ = normY * bboxHeight * distance / obj.focalLength(2); % Position Z calculée en fonction de la hauteur de l'image

                    % Position de l'objet dans l'espace de la caméra
                    objPositionCamera = [objX; objY; objZ];

                    % Appliquer la rotation et la translation pour obtenir la position globale
                    objectPos = translationMatrix + rotationMatrix * objPositionCamera;

                    % Stocker la position de l'objet et ses dimensions moyennes
                    objectXYZ(i, :) = objectPos';
                    objectDims(i, :) = [realDepth, realWidth, realHeight];

                    % Annoter l'image
                    labels{i} = sprintf('%s (%.2f m)', labels{i}, distance);

                    % Déterminer la couleur en fonction de la distance
                    if distance > obj.Offsetpersistent
                        objectColors(i) = 'green';
                        % Ajouter aux positions persistantes si la distance est supérieure à 3 m
                        idx = find(~obj.persistentObjectXYZ(:, 1), 1); % Trouver le premier indice vide
                        if ~isempty(idx)
                            obj.persistentObjectXYZ(idx, :) = objectPos';
                            obj.persistentObjectDims(idx, :) = [realDepth, realWidth, realHeight];
                        end
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
            objectDims = objectDims(nonZeroIdx, :);

            % Remplir les variables avec des zéros pour assurer une taille fixe de [100, 3]
            objectXYZ = [objectXYZ; zeros(100 - size(objectXYZ, 1), 3)];
            objectDims = [objectDims; zeros(100 - size(objectDims, 1), 3)];

            % Supprimer les lignes non utilisées des objets persistants
            nonZeroIdx = any(obj.persistentObjectXYZ ~= 0, 2);
            persistentObjectXYZ = obj.persistentObjectXYZ(nonZeroIdx, :);
            persistentObjectDims = obj.persistentObjectDims(nonZeroIdx, :);

            % Remplir les variables persistantes avec des zéros pour assurer une taille fixe de [100, 3]
            persistentObjectXYZ = [persistentObjectXYZ; zeros(100 - size(persistentObjectXYZ, 1), 3)];
            persistentObjectDims = [persistentObjectDims; zeros(100 - size(persistentObjectDims, 1), 3)];
        end

        %% Réinitialisation
        function resetImpl(obj)
            % Réinitialiser les objets persistants
            obj.persistentObjectXYZ = zeros(100, 3);
            obj.persistentObjectDims = zeros(100, 3);
        end

        %% Sauvegarder l'état
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            s.persistentObjectXYZ = obj.persistentObjectXYZ;
            s.persistentObjectDims = obj.persistentObjectDims;
        end

        %% Charger l'état
        function loadObjectImpl(obj, s, wasLocked)
            loadObjectImpl@matlab.System(obj, s, wasLocked);
            obj.persistentObjectXYZ = s.persistentObjectXYZ;
            obj.persistentObjectDims = s.persistentObjectDims;
        end

        %% Définir les tailles des sorties
        function [out1, out2, out3, out4, out5] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [100, 3]; % objectXYZ
            out2 = [100, 3]; % objectDims
            out3 = [100, 3]; % persistentObjectXYZ
            out4 = [100, 3]; % persistentObjectDims
            out5 = [720, 1280, 3]; % annotatedImage
        end

        %% Définir les types de données des sorties
        function [out1, out2, out3, out4, out5] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = 'double'; % objectXYZ
            out2 = 'double'; % objectDims
            out3 = 'double'; % persistentObjectXYZ
            out4 = 'double'; % persistentObjectDims
            out5 = 'uint8'; % annotatedImage
        end

        %% Définir si les sorties sont complexes
        function [out1, out2, out3, out4, out5] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false; % objectXYZ
            out2 = false; % objectDims
            out3 = false; % persistentObjectXYZ
            out4 = false; % persistentObjectDims
            out5 = false; % annotatedImage
        end

        %% Définir si les tailles des sorties sont fixes
        function [out1, out2, out3, out4, out5] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = false; % objectXYZ
            out2 = false; % objectDims
            out3 = false; % persistentObjectXYZ
            out4 = false; % persistentObjectDims
            out5 = true; % annotatedImage
        end

        %% Définir les noms des ports d'entrée
        function [name1, name2, name3] = getInputNamesImpl(~)
            % Return input port names for System block
            name1 = 'Image';
            name2 = 'TranslationMatrix';
            name3 = 'RotationMatrix';
        end

        %% Définir les noms des ports de sortie
        function [name1, name2, name3, name4, name5] = getOutputNamesImpl(~)
            % Return output port names for System block
            name1 = 'ObjectXYZ';
            name2 = 'ObjectDims';
            name3 = 'PersistentObjectXYZ';
            name4 = 'PersistentObjectDims';
            name5 = 'AnnotatedImage';
        end
    end
end
