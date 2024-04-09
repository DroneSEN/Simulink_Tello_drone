classdef Yolo_Object_Detector < matlab.System
    % untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
        
    end

    % Pre-computed constants or internal states
    properties (Access = private)
        yolo
    end

    methods (Access = protected)
        %Constructor

        function setupImpl(obj)
            name = 'tiny-yolov4-coco';
            % Perform one-time calculations, such as computing constants
            obj.yolo = yolov4ObjectDetector(name);%initialiser le yolo
        end

        function detectedImg = stepImpl(obj,I)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            [bboxs,scores,labels] = detect(obj.yolo,I, Threshold=0.4); %boxes/treshold(false positive)
            detectedImg = insertObjectAnnotation(I,"Rectangle",bboxs,labels);
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
        end
            

        function loadObjectImpl(obj,s,wasLocked)
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        function out1 = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [720 960 3];
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
