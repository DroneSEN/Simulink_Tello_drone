
% Script pour tracer les objets détectés sous forme de cubes, les positions des drones, et les points SLAM

droneGyroposition = out.droneGyroPositions;
droneSlamPositions = out.droneSlamPositions;
groundtruthopti = out.groundtruthopti;
objectPositions = out.objectPositions;
objectTypes = "tvmonitor";

%slamPoints = slamPoints;
%slamPoints_to_opti =[slamPoints(1);-slamPoints(2);slamPoints(3)];

% Charger les dimensions des objets depuis le fichier
data = load('objectDimensions.mat');
objectDimensions = data.objectDimensions;


% Créer une figure
figure;
hold on;


plot3(groundtruthopti(:, 1), groundtruthopti(:, 3), groundtruthopti(:, 2), 'r-', 'LineWidth', 1, 'DisplayName', 'True Position');
plot3(droneSlamPositions(:, 1), droneSlamPositions(:, 3), droneSlamPositions(:, 2), 'g-', 'LineWidth', 2, 'DisplayName', 'SLAM Position');
plot3(droneGyroposition(:, 1), droneGyroposition(:, 3), droneGyroposition(:, 2), 'o-', 'LineWidth', 2, 'DisplayName', 'Gyro Position');
% scatter3(slamPoints(:, 1), slamPoints(:, 3), slamPoints(:, 2), 'r.', 'DisplayName', 'SLAM Points');
scatter3(objectPositions(:, 1), objectPositions(:, 3), objectPositions(:, 2), 'b.', 'DisplayName', 'objectPositions');


% Personnaliser le tracé
xlabel('X');
ylabel('Y');
zlabel('Z'); %repère optitrack 
title('Objets détectés, trajectoire du drone et points SLAM en espace 3D');
grid on;
axis equal;

% Fixer la taille du graphique et centrer l'origine à (0,0,0)
xlim([-7, 7]);
ylim([-7, 7]);
zlim([0, 5]);

% Démarrer le drone au centre du graphique
view(3);
hold off;


