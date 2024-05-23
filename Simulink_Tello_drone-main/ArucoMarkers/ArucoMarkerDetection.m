classdef ArucoMarkerDetection < matlab.System & matlab.system.mixin.Propagates
    % ArucoMarkerDetection Detect ArUco markers and calculate camera position and orientation
    
    properties
        % Public, tunable properties
        markerSizeInMM = 150;
        markerFamily = 'DICT_4X4_250';
        maxMarkers = 1; % Maximum number of detectable markers
        
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
        
        function [outputImage, cameraPositions_repere_optitrack, eulerAngles] = stepImpl(obj, I)
            % Undistort the image
            I = undistortImage(I, obj.intrinsics);
            
            % Estimate marker poses
            [ids, locs, poses] = readArucoMarker(I, obj.markerFamily, obj.intrinsics, obj.markerSizeInMM);
            
            % Initialize output variables
            outputImage = I;
            cameraPositions = NaN(obj.maxMarkers, 3);
            eulerAngles = NaN(obj.maxMarkers, 3);
            
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
                
                % Calculate Euler angles from rotation matrix
                eulerAngles(i, :) = rotm2eul(poses(i).Rotation, 'ZYX');
            end
            
            % Convert camera positions to meters
            cameraPositions_metre_XYZ = cameraPositions / 1000; % Convert mm to meters
            %Y vers -Z
            %Z vers Y
            %X vers X
            cameraPositions_repere_optitrack = [cameraPositions_metre_XYZ(1) cameraPositions_metre_XYZ(3) -cameraPositions_metre_XYZ(2)];

            % Fill unused slots with zeros
            for i = (length(poses)+1):obj.maxMarkers
                cameraPositions_metre_XYZ(i, :) = [0, 0, 0];
                eulerAngles(i, :) = [0, 0, 0];
            end
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [outputImage, cameraPositions_repere_optitrack, eulerAngles] = getOutputSizeImpl(obj)
            % Return size for each output port
            outputImage = [720, 960, 3];
            cameraPositions_repere_optitrack = [obj.maxMarkers, 3];  % Fixed size array for positions
            eulerAngles = [obj.maxMarkers, 3];  % Fixed size array for Euler angles
        end
        
        function [outputImage, cameraPositions_repere_optitrack, eulerAngles] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            outputImage = 'uint8';
            cameraPositions_repere_optitrack = 'double';
            eulerAngles = 'double';
        end
        
        function [outputImage, cameraPositions_repere_optitrack, eulerAngles] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            outputImage = false;
            cameraPositions_repere_optitrack = false;
            eulerAngles = false;
        end
        
        function [outputImage, cameraPositions_repere_optitrack, eulerAngles] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            outputImage = true;
            cameraPositions_repere_optitrack = true;  % Fixed size for positions
            eulerAngles = true;  % Fixed size for Euler angles
        end
    end
end