% Script to plot detected objects as cubes, drone position, and SLAM points

% Load the object positions from the workspace
objectData = objectPositions;
objectTypes = objectTypes; % Load the object types from the workspace
dronePath = dronePositions; % Load the drone positions (X, Y, Z)
slamPoints = slamPoints; % Load the SLAM points (X, Y, Z)

% Load the object dimensions from the file
data = load('objectDimensions.mat');
objectDimensions = data.objectDimensions;

% Extract the time vector and data from the objectData structure
time = objectData.time;
positions = objectData.signals.values;
types = objectTypes.signals.values;

% Define transparency of the cubes
alphaValue = 0.5;

% Create a figure
figure;
hold on;

% Plot the SLAM points
scatter3(slamPoints(:, 1), slamPoints(:, 2), slamPoints(:, 3), 'r.');

% Plot the drone path
plot3(dronePath(:, 1), dronePath(:, 2), dronePath(:, 3), 'b-', 'LineWidth', 2);

% Loop through the positions and plot the cubes
for i = 1:length(time)
    pos = positions(i, :);
    objType = types{i};  % Get the type of object
    
    % Get the dimensions of the current object
    if isfield(objectDimensions, objType)
        dimensions = objectDimensions.(objType);
        cubeSize = dimensions;  % Size of the cube (width, height, depth)
    else
        % If dimensions are not found, use default values
        cubeSize = [1, 1, 1];
    end
    
    % Calculate the origin of the cube based on the center position
    origin = pos - cubeSize / 2;
    
    % Plot the cube at the given position with the actual size
    plotcube(cubeSize, origin, alphaValue, [0 1 0]);  % Green color
end

% Customize the plot
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Detected Objects, Drone Path, and SLAM Points in 3D Space');
grid on;
axis equal;

% Fix the graph size and center the origin at (0,0,0)
xlim([-2.5, 2.5]);
ylim([-2.5, 2.5]);
zlim([-1, 1]);

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

% Default input arguments
inArgs = {[10 56 100], [10 10 10], 0.7, [1 0 0]};

% Replace default input arguments by input values
inArgs(1:nargin) = varargin;

% Create all variables
[edges, origin, alpha, clr] = deal(inArgs{:});

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
    repmat({clr}, 6, 1), ...
    repmat({'FaceAlpha'}, 6, 1), ...
    repmat({alpha}, 6, 1));

view(3);
end
