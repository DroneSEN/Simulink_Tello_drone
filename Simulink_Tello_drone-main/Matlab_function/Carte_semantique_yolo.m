% Chargement des données, si disponibles
if exist('out', 'var')
    if isfield(out, 'groundtruthopti')
        groundtruthopti = out.groundtruthopti;
    end
    if isfield(out, 'objectPositions')
        objectPositions = out.objectPositions;
    end
end

%Chargement Slam/gyro
%droneGyroposition = out.droneGyroPositions;
%droneSlamPositions = out.droneSlamPositions;

% Créer une figure en plein écran
figure('WindowState', 'maximized');
hold on;

posobj1 = [-1 1 2];
posobj2 = [1 1 2];
objdimension = [0.47 0.2 0.33];

% Tracement des cubes pour les positions des objets
plotcube(objdimension, posobj1, 0.8, [0 1 0], 'Premier Objet');  % Couleur verte
plotcube(objdimension, posobj2, 0.8, [0 1 0], 'Second Objet');  % Couleur verte

% Tracement des positions estimées, si disponibles
if exist('objectPositions', 'var')
    scatter3(objectPositions(:, 1), objectPositions(:, 3), objectPositions(:, 2), 'b.', 'DisplayName', 'objectPositions');
end

% Tracement de la position groundtruthopti, si disponible
if exist('groundtruthopti', 'var')
    plot3(groundtruthopti(:, 1), groundtruthopti(:, 3), groundtruthopti(:, 2), 'r-', 'LineWidth', 1, 'DisplayName', 'True Position');
end

% Tracement de la position slam/gyro/xyzslampoint
%plot3(droneSlamPositions(:, 1), droneSlamPositions(:, 3), droneSlamPositions(:, 2), 'g-', 'LineWidth', 2, 'DisplayName', 'SLAM Position');
%plot3(droneGyroposition(:, 1), droneGyroposition(:, 3), droneGyroposition(:, 2), 'o-', 'LineWidth', 2, 'DisplayName', 'Gyro Position');
% scatter3(slamPoints(:, 1), slamPoints(:, 3), slamPoints(:, 2), 'r.', 'DisplayName', 'SLAM Points');







% Personnaliser le tracé
xlabel('X');
ylabel('Y');
zlabel('Z'); %repère optitrack
title('Objets détectés, trajectoire du drone et points SLAM en espace 3D');
grid on;
axis equal;

% Fixer la taille du graphique et centrer l'origine à (0,0,0)
xlim([-4, 4]);
ylim([-4, 4]);
zlim([0, 3]);

% Afficher la légende
legend('show');

% Démarrer le drone au centre du graphique
view(3);
hold off;

% Définition de la fonction plotcube
function plotcube(edges, origin, alpha, color, displayName)
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
        repmat({alpha}, 6, 1), ...
        repmat({'DisplayName'}, 6, 1), ...
        repmat({displayName}, 6, 1));  % Ajout de la légende

    view(3);
end
