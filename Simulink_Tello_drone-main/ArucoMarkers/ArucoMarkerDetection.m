classdef ArucoMarkerDetection < matlab.System & matlab.system.mixin.Propagates
    % ArucoMarkerDetection Detect ArUco markers and calculate camera position
    
    properties
        % Public, tunable properties
        markerSizeInMM = 150;
        markerFamily = 'DICT_4X4_250';
        maxMarkers = 10; % Maximum number of detectable markers
        
        % Camera intrinsic parameters
        focalLength;
        principalPoint;
        imageSize;
    end
    
    properties(Access = private)
        % Pre-computed constants
        intrinsics;  % Camera intrinsics object
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Set camera intrinsics using the provided properties
            obj.intrinsics = cameraIntrinsics(obj.focalLength, obj.principalPoint, obj.imageSize);
        end
        
        function [outputImage, cameraPositions_metre_XYZ] = stepImpl(obj, I)
            % Undistort the image
            I = undistortImage(I, obj.intrinsics);
            
            % Estimate marker poses
            [ids, locs, poses] = readArucoMarker(I, obj.markerFamily, obj.intrinsics, obj.markerSizeInMM);
            
            % Initialize output variables
            outputImage = I;
            cameraPositions = NaN(obj.maxMarkers, 3);
            
            % Points for the object coordinate system
            worldPoints = [0 0 0; obj.markerSizeInMM/2 0 0; 0 obj.markerSizeInMM/2 0; 0 0 obj.markerSizeInMM/2];

            for i = 1:length(poses)
                if i > obj.maxMarkers
                    break;
                end
                % Get image coordinates for axes
                imagePoints = worldToImage(obj.intrinsics, poses(i).Rotation, poses(i).Translation, worldPoints);
                
                % Points for axes
                axesPoints = [imagePoints(1,:) imagePoints(2,:);
                              imagePoints(1,:) imagePoints(3,:);
                              imagePoints(1,:) imagePoints(4,:)];
                
                % Draw colored axes
                outputImage = insertShape(outputImage, "Line", axesPoints, ...
                    'Color', ["red","green","blue"], 'LineWidth', 10);
                
                % Calculate camera position relative to the marker
                cameraPositions(i, :) = -poses(i).Translation * poses(i).Rotation';
            end
            
            % Convert camera positions to meters
            cameraPositions_metre_XYZ = cameraPositions / 1000; % Convert mm to meters
            
            % Fill unused slots with zeros
            for i = (length(poses)+1):obj.maxMarkers
                cameraPositions_metre_XYZ(i, :) = [0, 0, 0];
            end
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [outputImage, cameraPositions_metre_XYZ] = getOutputSizeImpl(obj)
            % Return size for each output port
            outputImage = [720, 960, 3];
            cameraPositions_metre_XYZ = [obj.maxMarkers, 3];  % Fixed size array for positions
        end
        
        function [outputImage, cameraPositions_metre_XYZ] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            outputImage = 'uint8';
            cameraPositions_metre_XYZ = 'double';
        end
        
        function [outputImage, cameraPositions_metre_XYZ] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            outputImage = false;
            cameraPositions_metre_XYZ = false;
        end
        
        function [outputImage, cameraPositions_metre_XYZ] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            outputImage = true;
            cameraPositions_metre_XYZ = true;  % Fixed size for positions
        end
    end
end
