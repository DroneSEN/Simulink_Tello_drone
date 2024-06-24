classdef Yolo_Object_Detector < matlab.System
    % Yolo_Object_Detector - Détecte des objets et calcule leur position en XYZ

    properties (Access = private)
        yolo                % Détecteur YOLO
        focalLength         % Longueur focale de la caméra
        objectDimensions    % Tailles moyennes des objets (hauteur, largeur, profondeur)
        targetObject        % Objet cible à détecter
        imageSize           % Taille de l'image [height, width]
        imageCenterPoint    % Point central de l'image
        lastKnownPos        % Dernière position connue de l'objet
        intrinsicMatrix     % Matrice intrinsèque (matrice de projection)
        objectPositions     % Positions des objets détectés
        objectTypes         % Types des objets détectés
        intrinsics;
    end

    methods (Access = protected)
        %% Initialisation
        function setupImpl(obj)
            % Initialiser le détecteur YOLO
            name = 'tiny-yolov4-coco';
            obj.yolo = yolov4ObjectDetector(name);

            % Définir la longueur focale de la caméra et le point principal (exemples de valeurs)
            obj.focalLength = [878, 883]; % [fx, fy]
            obj.imageCenterPoint = [488, 363]; % [cx, cy]

            % Définir la taille de l'image [height, width] en pixels
            obj.imageSize = [720, 960];

            % Charger les dimensions moyennes des objets à partir d'un fichier
            data = load('objectDimensions.mat');
            obj.objectDimensions = data.objectDimensions;

            % Définir l'objet cible (initialisation)
            obj.targetObject = 'tvmonitor';

            % Initialiser la dernière position connue
            obj.lastKnownPos = [NaN; NaN; NaN; NaN];  % 4x1
            obj.intrinsics = cameraIntrinsics(obj.focalLength, obj.imageCenterPoint, obj.imageSize);

            % Définir la matrice intrinsèque (de projection) basée sur la longueur focale et le centre de l'image
            obj.intrinsicMatrix = [obj.focalLength(1), 0, obj.imageCenterPoint(1);
                                   0, obj.focalLength(2), obj.imageCenterPoint(2);
                                   0, 0, 1];

            % Initialiser les propriétés pour stocker les positions et types d'objets
            obj.objectPositions = [];
            obj.objectTypes = {};
        end

        %% Détection des objets et calcul de leurs positions
        function [objectPos, annotatedImage, bboxDimensions, camCoordinates, point_cam, point_cam_todrone] = stepImpl(obj, I)
            % I : image d'entrée
            % tform : matrice de transformation du repère caméra au repère monde (entrée)

            I = undistortImage(I, obj.intrinsics);

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

            % Initialiser la position de l'objet et l'image annotée
            annotatedImage = I;

            % Initialiser les sorties supplémentaires
            bboxDimensions = [NaN, NaN]; % [w, h]
            camCoordinates = [NaN, NaN]; % [X_cam, Z_cam]
            point_cam = [NaN; NaN; NaN; NaN];
            point_cam_todrone = [NaN; NaN; NaN; NaN];

            % Si aucun objet n'est détecté, retourner la dernière position connue et NaN pour les nouvelles sorties
            if isempty(bboxs)
                objectPos = obj.lastKnownPos;
                return;
            end

            % Si plusieurs objets sont détectés, retourner la dernière position connue et NaN pour les nouvelles sorties
            if size(bboxs, 1) > 1
                objectPos = obj.lastKnownPos;
                return;
            end

            % Définir les variables pour le calcul des positions
            x = bboxs(1, 1);
            y = bboxs(1, 2);
            w = bboxs(1, 3);
            h = bboxs(1, 4);

            % Stocker les coordonnées caméra
            camCoordinates = [x, y];
            camCoordinates = double(camCoordinates);

            % Stocker les dimensions de la boîte englobante
            bboxDimensions = [w, h];
            bboxDimensions = double(bboxDimensions);

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
                Dh = (obj.focalLength(2) * realHeight) / h; % Utiliser fy pour la hauteur
                Dw = (obj.focalLength(1) * realWidth) / w;  % Utiliser fx pour la largeur
                D = (Dh + Dw) / 2;

                % Ajuster la distance avec la moitié de la profondeur de l'objet
                Distance = D + realDepth / 2;

                % Calcul des coordonnées dans le référentiel de la caméra
                X_cam = (x_center - obj.imageCenterPoint(1)) * Distance / obj.focalLength(1);
                Y_cam = (y_center - obj.imageCenterPoint(2)) * Distance / obj.focalLength(2);
                Z_cam = Distance;  % En supposant que la caméra est alignée avec l'axe Z
                

                %Coordonnées camera selon model camera pinhole
                point_cam = [X_cam; Y_cam; Z_cam; 1];
                theta = -10 * pi / 180;  % Exemple d'inclinaison de 10 degrés vers le bas
                Rx = [1 0 0 0; 0 cos(theta) -sin(theta) 0; 0 sin(theta) cos(theta) 0; 0 0 0 1];  % Matrice de rotation X
                P = [0 0 1 0; 1 0 0 0; 0 1 0 0; 0 0 0 1];  % Matrice de permutation
                Tform = P * Rx;  % Combinaison de la rotation et de la permutation

                %Coordonnées camtodrone selon model camera pinhole
                %point_cam_todrone = [Z_cam; X_cam; Y_cam; 1];
                % Application de la transformation de la caméra au drone
                point_cam_todrone = Tform * point_cam;

                % Transformation vers le référentiel monde
                point_cam = double([point_cam(1:3); 1]);  % Assurez-vous que point_cam est 4x1
                point_cam_todrone = double([point_cam_todrone(1:3); 1]);  % Assurez-vous que point_cam_todrone est 4x1

                % Annoter l'image avec la distance et la position
                label = sprintf('%s (%.2f m, [%.2f, %.2f, %.2f])', obj.targetObject, D, point_cam_todrone(1), point_cam_todrone(2), point_cam_todrone(3));
                annotatedImage = insertObjectAnnotation(I, "rectangle", bboxs(1, :), label, 'Color', 'yellow');
                
                % Mettre à jour la dernière position connue
                obj.lastKnownPos = point_cam_todrone;
            end
            
            % Retourner la position mise à jour
            objectPos = obj.lastKnownPos;
        end

        %% Définir les tailles des sorties
        function [out1, out2, out3, out4, point_cam, point_cam_todrone] = getOutputSizeImpl(~)
            % Retourner la taille de chaque port de sortie
            out1 = [4, 1]; % objectPos
            out2 = [720, 960, 3]; % annotatedImage (exemple de taille d'image)
            out3 = [1, 2]; % bboxDimensions
            out4 = [1, 2]; % camCoordinates
            point_cam = [4, 1];
            point_cam_todrone =  [4, 1]; 
        end

        %% Définir les types de données des sorties
        function [out1, out2, out3, out4, point_cam, point_cam_todrone] = getOutputDataTypeImpl(~)
            % Retourner le type de données de chaque port de sortie
            out1 = 'double'; % objectPos
            out2 = 'uint8'; % annotatedImage
            out3 = 'double'; % bboxDimensions
            out4 = 'double'; % camCoordinates
            point_cam = 'double';
            point_cam_todrone = 'double';
        end

        %% Définir si les sorties sont complexes
        function [out1, out2, out3, out4, point_cam, point_cam_todrone] = isOutputComplexImpl(~)
            % Retourner vrai pour chaque port de sortie avec des données complexes
            out1 = false; % objectPos
            out2 = false; % annotatedImage
            out3 = false; % bboxDimensions
            out4 = false; % camCoordinates
            point_cam = false;
            point_cam_todrone = false;
        end

        %% Définir si les tailles des sorties sont fixes
        function [out1, out2, out3, out4, point_cam, point_cam_todrone] = isOutputFixedSizeImpl(~)
            % Retourner vrai pour chaque port de sortie avec une taille fixe
            out1 = true; % objectPos
            out2 = true; % annotatedImage
            out3 = true; % bboxDimensions
            out4 = true; % camCoordinates
            point_cam = true;
            point_cam_todrone = true;
        end

        %% Définir les noms des ports d'entrée
        function [name1, name2] = getInputNamesImpl(~)
            % Retourner les noms des ports d'entrée pour le bloc système
            name1 = 'Image';
            name2 = 'Tform';
        end

        %% Définir les noms des ports de sortie
        function [name1, name2, name3, name4, point_cam, point_cam_todrone] = getOutputNamesImpl(~)
            % Retourner les noms des ports de sortie pour le bloc système
            name1 = 'ObjectPos';
            name2 = 'AnnotatedImage';
            name3 = 'BBoxDimensions';
            name4 = 'CamCoordinates';
            point_cam = 'posecam';
            point_cam_todrone = 'posecamtodrone';
        end

        %% Définir le temps d'échantillonnage
        function sts = getSampleTimeImpl(obj)
            % Définir le type de temps d'échantillonnage
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', 1);
        end
    end
end
