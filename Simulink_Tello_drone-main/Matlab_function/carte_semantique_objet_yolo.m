
% Script pour tracer les objets détectés sous forme de cubes, les positions des drones, et les points SLAM

% droneGyroposition = out.droneGyroPositions;
% droneSlamPositions = out.droneSlamPositions;
groundtruthopti = out.groundtruthopti;
objectPositions = out.objectPositions;
objectTypes = "tvmonitor";


% Initialisation du tableau filtré avec la première ligne
filteredPositionsobj = objectPositions(1, :);

% Seuil de différence en cm (0.5 m)
threshold = 0.1;



% Charger les dimensions des objets depuis le fichier
data = load('objectDimensions.mat');
objectDimensions = data.objectDimensions;

% Définir la transparence des cubes
alphaValue = 0.8;

% Créer une figure
figure;
hold on;
% 
% % Tracer les objets
% if ~isempty(objectPositions)
%     for i = 1:size(objectPositions, 1)
%         pos = objectPositions(i, :);
% 
% 
%         % Obtenir les dimensions de l'objet courant
%         if isfield(objectDimensions, objectTypes)
%             dimensions = objectDimensions.(objectTypes);
%             cubeSize = [dimensions(3), dimensions(2), dimensions(1)];  % Mapper les dimensions à [hauteur, largeur, profondeur]
%         else
%             % Si les dimensions ne sont pas trouvées, utiliser des valeurs par défaut
%             cubeSize = [0,0, 0];
%         end
% 
%         % Calculer l'origine du cube basée sur la position centrale
%         origin = pos - cubeSize / 2;
% 
%         % Tracer le cube à la position donnée avec la taille réelle
%         plotcube(cubeSize, origin, alphaValue, [0 1 0]);  % Couleur verte
%     end
% end
% 
% cubeSizedrone = [10,5,10];
% plotcube(cubeSizedrone, groundtruthopti, alphaValue, [0 0 1]);  % Couleur verte

plot3(groundtruthopti(:, 1), groundtruthopti(:, 3), groundtruthopti(:, 2), 'r-', 'LineWidth', 1, 'DisplayName', 'True Position');
% plot3(droneSlamPositions(:, 1), droneSlamPositions(:, 3), droneSlamPositions(:, 2), 'r-', 'LineWidth', 2, 'DisplayName', 'SLAM Position');
%plot3(droneGyroposition(:, 1), droneGyroposition(:, 3), droneGyroposition(:, 2), 'g-', 'LineWidth', 2, 'DisplayName', 'Gyro Position');
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
xlim([-4, 4]);
ylim([-4, 4]);
zlim([0, 5]);

% Démarrer le drone au centre du graphique
view(3);
hold off;

function plotcube(edges, origin, alpha, color)
    % PLOTCUBE - Affiche un cube 3D dans les axes courants
    %
    %   PLOTCUBE(EDGES, ORIGIN, ALPHA, COLOR) affiche un cube 3D dans les axes courants
    %   avec les propriétés suivantes :
    %   * EDGES : vecteur de 3 éléments qui définit la longueur des arêtes du cube
    %   * ORIGIN: vecteur de 3 éléments qui définit le point de départ du cube
    %   * ALPHA : scalaire qui définit la transparence des faces du cube (de 0
    %             à 1)
    %   * COLOR : vecteur de 3 éléments qui définit la couleur des faces du cube
    %
    % Exemple :
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

for i = 2:size(objectPositions, 1)
    % Calculer la différence absolue entre les positions actuelles et précédentes
    diff = abs(objectPositions(i, :) - objectPositions(i - 1, :));
    
    % Vérifier si la différence est supérieure au seuil pour toutes les coordonnées
    if any(diff > threshold)
        filteredPositions = [filteredPositions; objectPositions(i, :)];
    end
end
