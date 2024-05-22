classdef ResNet50ImageClassifier < matlab.System
    % ResNet50ImageClassifier - MATLAB System to classify images using ResNet-50
    
    % Properties
    properties
        ModelPath = "resnet50"; % Default ResNet-50 model
        CameraID = 1; % ID de la caméra à utiliser
    end
    
    % Pre-computed constants or internal states
    properties (Access = private)
        net
        classes
        cam
    end

    methods
        % Constructor
        function obj = ResNet50ImageClassifier()
            % Load the pretrained ResNet-50 model
            [obj.net, obj.classes] = imagePretrainedNetwork(obj.ModelPath);
            
            % Initialize webcam
            obj.cam = webcam(obj.CameraID);
        end
    end

    methods (Access = protected)
        % Implementation of step method
        function labeledImage = stepImpl(obj, ~)
            % Capture image from webcam
            I = snapshot(obj.cam);
            
            % Adjust size of the image if needed
            sz = obj.net.Layers(1).InputSize;
            I = I(1:sz(1), 1:sz(2), 1:sz(3));
            
            % Classify the image using ResNet-50
            scores = predict(obj.net, single(I));
            [~, idx] = max(scores);
            label = obj.classes(idx);
            
            % Show the image and the classification results
            labeledImage = insertText(I, [10, 20], char(label), 'TextColor', 'white');
        end
        
        % Define output size
        function sz = getOutputSizeImpl(~)
            sz = [224 224 3];
        end

        % Define output data type
        function dt = getOutputDataTypeImpl(~)
            dt = 'uint8';
        end

        % Define complexity of output
        function cplx = isOutputComplexImpl(~)
            cplx = false;
        end

        % Define fixed-size or variable-size output
        function fs = isOutputFixedSizeImpl(~)
            fs = true;
        end
    end
end
