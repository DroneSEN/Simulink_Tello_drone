classdef cameramac < matlab.System
    % camera Add summary here

    % Public, tunable properties
    properties
    
    end

    % Pre-computed constants or internal states
    properties (Access = private)
        cam;
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.cam = webcam(1); % Utiliser l'index de la caméra
        end

        function img = stepImpl(obj)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            img = snapshot(obj.cam);
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function num = getNumOutputsImpl(~)
            % Return the number of outputs
            num = 1;
        end

        function flag = isOutputFixedSizeImpl(~, ~)
            % Return true if output is fixed-size
            flag = true;
        end

        function size = getOutputSizeImpl(~)
            % Return size for each output port
            size = [720, 1280, 3]; % Taille de l'image
        end

        function dataType = getOutputDataTypeImpl(~)
            % Return data type for each output port
            dataType = 'uint8';
        end

        function complex = isOutputComplexImpl(~)
            % Return true if output is complex
            complex = false;
        end

        function isDirectFeedthrough = isInputDirectFeedthroughImpl(~)
            % Return true if System object has direct feedthrough
            isDirectFeedthrough = false;
        end
    end
end
