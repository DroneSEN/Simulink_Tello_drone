% Script to plot detected objects as cubes, drone positions, and SLAM points

% Load the object positions and types from the workspace
if evalin('base', 'exist(''objectPositions'', ''var'')')
    objectPositions = evalin('base', 'objectPositions');
    objectTypes = evalin('base', 'objectTypes');
else
    objectPositions = [];
    objectTypes = {};
end

% Load the object dimensions from the file
data = load('objectDimensions.mat');
objectDimensions = data.objectDimensions;

% Define transparency of the cubes
alphaValue = 0.5;

% Create a figure
figure;
hold on;

% Plot the objects
if ~isempty(objectPositions)
    for i = 1:size(objectPositions, 1)
        pos = objectPositions(i, :);
        objType = objectTypes{i};  % Get the type of object
        
        % Get the dimensions of the current object
        if isfield(objectDimensions, objType)
            dimensions = objectDimensions.(objType);
            cubeSize = [dimensions(2), dimensions(3), dimensions(1)];  % Map dimensions to [height, width, depth]
        else
            % If dimensions are not found, use default values
            cubeSize = [1, 1, 1];
        end
        
        % Calculate the origin of the cube based on the center position
        origin = pos - cubeSize / 2;
        
        % Plot the cube at the given position with the actual size
        plotcube(cubeSize, origin, alphaValue, [0 1 0]);  % Green color
    end
end

% Plot the drone positions
if evalin('base', 'exist(''groundtruthopti'', ''var'')')
    groundtruthopti = evalin('base', 'groundtruthopti');
    plot3(groundtruthopti(:, 1), groundtruthopti(:, 3), groundtruthopti(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', 'True Position');
end

if evalin('base', 'exist(''droneSlamPositions'', ''var'')')
    droneSlamPositions = evalin('base', 'droneSlamPositions');
    plot3(droneSlamPositions(:, 1), droneSlamPositions(:, 3), droneSlamPositions(:, 2), 'r-', 'LineWidth', 2, 'DisplayName', 'SLAM Position');
end

if evalin('base', 'exist(''droneGyroPositions'', ''var'')')
    droneGyroPositions = evalin('base', 'droneGyroPositions');
    plot3(droneGyroPositions(:, 1), droneGyroPositions(:, 3), droneGyroPositions(:, 2), 'g-', 'LineWidth', 2, 'DisplayName', 'Gyro Position');
end

% Plot the SLAM points
if evalin('base', 'exist(''slamPoints'', ''var'')')
    slamPoints = evalin('base', 'slamPoints');
    scatter3(slamPoints(:, 1), slamPoints(:, 3), slamPoints(:, 2), 'r.', 'DisplayName', 'SLAM Points');
end

% Customize the plot
xlabel('X');
ylabel('Z');
zlabel('Y');%repère optitrack 
title('Detected Objects, Drone Path, and SLAM Points in 3D Space');
grid on;
axis equal;

% Fix the graph size and center the origin at (0,0,0)
xlim([-5, 5]);
ylim([-5, 5]);
zlim([0, 2]);

% Start the drone at the center of the graph
view(3);
hold off;

function plotcube(edges, origin, alpha, color)
    % PLOTCUBE - Display a 3D-cube in the current axes
    %
    %   PLOTCUBE(EDGES, ORIGIN, ALPHA, COLOR) displays a 3D-cube in the current axes
    %   with the following properties:
    %   * EDGES : 3-elements vector that defines the length of cube edges
    %   * ORIGIN: 3-elements vector that defines the start point of the cube
    %   * ALPHA : scalar that defines the transparency of the cube faces (from 0
    %             to 1)
    %   * COLOR : 3-elements vector that defines the faces color of the cube
    %
    % Example:
    %   >> plotcube([5 5 5], [2 2 2], 0.8, [1 0 0]);
    %   >> plotcube([5 5 5], [10 10 10], 0.8, [0 1 0]);
    %   >> plotcube([5 5 5], [20 20 20], 0.8, [0 0 1]);

    XYZ = {...
        [0 0 0 0]  [0 0 1 1]  [0 1 1 0]; ...
        [1 1 1 1]  [0 0 1 1]  [0 1 1 0]; ...
        [0 1 1 0]  [0 0 0 0]  [0 0 1 1]; ...
        [0 1 1 0]  [1 1 1 1]  [0 0 1 1]; ...
        [0 1 1 0]  [0 0 1 1]  [0 0 0 0]; ...
        [0 1 1 0]  [0 0 1 1]  [1 1 1 1] ...
        };

    XYZ = mat2cell(...
        cellfun(@(x,y,z) x*y+z, ...
        XYZ, ...
        repmat(mat2cell(edges,1,[1 1 1]),6,1), ...
        repmat(mat2cell(origin,1,[1 1 1]),6,1), ...
        'UniformOutput', false), ...
        6, [1 1 1]);

    cellfun(@patch, XYZ{1}, XYZ{2}, XYZ{3}, ...
        repmat({color}, 6, 1), ...
        repmat({'FaceAlpha'}, 6, 1), ...
        repmat({alpha}, 6, 1));

    view(3);
end
