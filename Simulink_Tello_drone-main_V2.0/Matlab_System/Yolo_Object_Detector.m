classdef Yolo_Object_Detector < matlab.System
    % Yolo_Object_Detector Add summary here

    % Public, tunable properties
    properties
        
    end

    % Pre-computed constants or internal states
    properties (Access = private)
        yolo
        focalLength % Longueur focale de la caméra
        realPersonHeight % Hauteur réelle d'une personne (en mètres)
    end

    methods (Access = protected)
        % Constructor
        function setupImpl(obj)
            name = 'tiny-yolov4-coco';
            % Perform one-time calculations, such as computing constants
            obj.yolo = yolov4ObjectDetector(name); % Initialiser le YOLO
            obj.focalLength = [1.002777899955502e+03, 1.006322192826368e+03];
            obj.realPersonHeight = 1.7; % Hauteur réelle d'une personne en mètres
        end

        function detectedImg = stepImpl(obj, I)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            [bboxs, scores, labels] = detect(obj.yolo, I, Threshold=0.4); % Boxes/threshold (false positive)
            
            % Convertir les labels en tableau de cellules
            if ~iscell(labels)
                labels = cellstr(labels);
            end
            
            % Calculer la distance pour chaque personne détectée et ajouter l'annotation
            for i = 1:size(bboxs, 1)
                if strcmp(labels{i}, 'person')
                    % Calculer la hauteur apparente de l'objet en pixels
                    apparentHeight = bboxs(i, 4);
                    
                    % Utiliser la formule pour calculer la distance
                    distance = (obj.realPersonHeight * obj.focalLength(2)) / apparentHeight;
                    
                    % Ajouter la distance à l'annotation
                    labels{i} = sprintf('Person (%.2f m)', distance);
                end
            end
            
            % Annoter l'image avec les boîtes et les distances
            detectedImg = insertObjectAnnotation(I, "rectangle", bboxs, labels);
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
        end

        function loadObjectImpl(obj, s, wasLocked)
            loadObjectImpl@matlab.System(obj, s, wasLocked);
        end

        function out1 = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [720, 1280, 3];
        end

        function out1 = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = 'uint8';
        end

        function out1 = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = false;
        end

        function out1 = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1 = true;
        end

        function name1 = getInputNamesImpl(obj)
            % Return input port names for System block
            name1 = 'Image';
        end

        function name1 = getOutputNamesImpl(obj)
            % Return output port names for System block
            name1 = 'Image';
        end

    end
end
