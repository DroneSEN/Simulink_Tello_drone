% MonoVisualSLAMSystem Monocular visual SLAM algorithm
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.

%   Copyright 2021-2022 The MathWorks, Inc.

classdef MonoVisualSLAMSystem < matlab.System

    % Public, non-tunable properties
    properties(Nontunable)
        %FocalLength Camera focal length
        FocalLength = [756.05 748.04];
        %PrincipalPoint Camera focal center
        PrincipalPoint = [495.48 418.80];
        %ImageSize Image size
        ImageSize = [720 960];
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
            numPoints = 2000;
            numSkipFrames = 10;
            obj.VslamObj = monovslam(intrinsics,MaxNumPoints=numPoints,SkipMaxFrames=numSkipFrames);
            obj.Pose = eye(4);
        end

        function [pose ,isTrackingLost,xyzPoints] = stepImpl(obj, I) %[pose, isTrackingLost] = 
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            addFrame(obj.VslamObj,I);
            while ~isDone(obj.VslamObj)
                if hasNewKeyFrame(obj.VslamObj)
                    plot(obj.VslamObj);
                    [camPoses,~] = poses(obj.VslamObj);
                    p = camPoses(end);
                    obj.Pose = p.A;
                end
            end
            pose = obj.Pose;
            isTrackingLost =~ uint8(checkStatus(obj.VslamObj));
            xyzPoints = mapPoints(obj.VslamObj);
          
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

        function [out1,out2,out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [4 4];
            out2 = [1 1];
            out3 = [10000 3]; % La première dimension est variable, la seconde est fixée à 3
        end

        function [out1,out2,out3] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = 'double';
            out2 = 'boolean';
            out3 = 'double';
        end

        function [out1, out2,out3] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
        end

        function [out1,out2,out3] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = false;
        end

        function [name1] = getInputNamesImpl(obj)
            % Return input port names for System block
            name1 = 'Image';
        end

        function [name1, name2,name3] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name1 = 'Camera Pose';
            name2 = 'Tracking Lost';
            name3 = 'XYZmappoints';
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
