% MonoVisualSLAMSystem Monocular visual SLAM algorithm
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.

%   Copyright 2021-2022 The MathWorks, Inc.

classdef MonoVisualSLAMSystem < matlab.System

    % Public, non-tunable properties
    properties(Nontunable)
        %FocalLength Camera focal length
        FocalLength    = [1109 1109]

        %PrincipalPoint Camera focal center
        PrincipalPoint = [640 360]

        %ImageSize Image size
        ImageSize      = [720 1280]

        numPoints = 1000;
        
        numSkipFrames = 20;
    end

    % Pre-computed constants
    properties(Access = private)
        VslamObj
        Pose (4,4) double
    end

    methods
        % Constructor
        function obj = MonoVisualSLAMSystem(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            intrinsics = cameraIntrinsics(obj.FocalLength, obj.PrincipalPoint, obj.ImageSize);
            obj.numPoints = 1000;
            obj.numSkipFrames = 20;
            obj.VslamObj = monovslam(intrinsics,MaxNumPoints=obj.numPoints,SkipMaxFrames=obj.numSkipFrames);
            obj.Pose = eye(4);
        end

        function [pose, isTrackingLost] = stepImpl(obj, I)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            addFrame(obj.VslamObj,I);

            if hasNewKeyFrame(obj.VslamObj)
                plot(obj.VslamObj);
                [camPoses,~] = poses(obj.VslamObj);
                p = camPoses(end);
                obj.Pose = p.A;
            end

            xyzPoints = mapPoints(obj.VslamObj);
            assignin('base', 'xyzPoints', xyzPoints);  % Envoyer xyzPoints au workspace global

            pose = obj.Pose;
            isTrackingLost =~ uint8(checkStatus(obj.VslamObj));
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
        end

        function loadObjectImpl(obj,s,wasLocked)
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
       
        %% Simulink functions
        % function ds = getDiscreteStateImpl(obj)
        %     % Return structure of properties with DiscreteState attribute
        %     ds = struct([]);
        % end

        % function flag = isInputSizeMutableImpl(obj,index)
        %     % Return false if input size cannot change
        %     % between calls to the System object
        %     flag = false;
        % end

        function [out1,out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [4 4];
            out2 = [1 1];
        end

        function [out1,out2] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = 'double';
            out2 = 'boolean';
        end

        function [out1, out2] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
        end

        function [out1,out2] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
        end

        function [name1] = getInputNamesImpl(obj)
            % Return input port names for System block
            name1 = 'Image';
        end

        function [name1, name2] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name1 = 'Camera Pose';
            name2 = 'Tracking Lost';
        end
    end

    methods(Static, Access = protected)
        %% Simulink customization functions
        % function header = getHeaderImpl
        %     % Define header panel for System block dialog
        %     header = matlab.system.display.Header(mfilename("class"));
        % end

        % function group = getPropertyGroupsImpl
        %     % Define property section(s) for System block dialog
        %     group = matlab.system.display.Section(mfilename("class"));
        % end

        % function flag = showSimulateUsingImpl
        %     % Return false if simulation mode hidden in System block dialog
        %     flag = true;
        % end
    end
end
