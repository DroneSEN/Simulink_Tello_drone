%% Chargement des variables
% Initialisation des variables à vide
groundtruthopti = [];
objectPositions = [];
droneSlamPositions = [];
xyzPoints = [];

% Chargement des données, si disponibles
if exist('out', 'var')
    try
        groundtruthopti = out.groundtruthopti;
    catch
        disp('groundtruthopti non disponible.');
    end

    try
        objectPositions = out.objectPositions;
    catch
        disp('objectPositions non disponible.');
    end

    try
        droneSlamPositions = out.droneSlamPositions;
    catch
        disp('droneSlamPositions non disponible.');
    end

    try
        xyzPoints = out.xyzPoints;
    catch
        disp('xyzPoints non disponible.');
    end
end


%droneGyroposition = out.droneGyroPositions;

%% Creation de la figure
% Créer une figure en plein écran
figure('WindowState', 'maximized');
hold on;

%% Objet connu sur la carte sémantique
%Dimension de l'objet connu
objdimension = [0.47 0.33 0.22]; %ordinateur portable

posobj1 = [0.8 1 2]; % repère opti X,Y,Z 
origin1 = posobj1 - objdimension / 2;
posobj2 = [-0.8 1 2]; % repère opti 
origin2 = posobj2 - objdimension / 2;

% Tracement des cubes pour les positions des objets
plotcube(objdimension, origin1, 0.8, [0 1 0], 'Premier Objet');  % Couleur verte
plotcube(objdimension, origin2, 0.8, [0 1 0], 'Second Objet');  % Couleur verte

%% Tracement
% Tracement des positions estimées, si disponibles
if ~isempty(objectPositions)
    scatter3(objectPositions(:, 1), objectPositions(:, 2), objectPositions(:, 3), 'r.', 'DisplayName', 'objectPositions');
end

% Tracement de la position groundtruthopti, si disponible
if ~isempty(groundtruthopti)
    plot3(groundtruthopti(:, 1), groundtruthopti(:, 2), groundtruthopti(:, 3), 'g-', 'LineWidth', 1, 'DisplayName', 'True Position');
end

% Tracement de la position droneSlamPositions, si disponible
if ~isempty(droneSlamPositions)
    plot3(droneSlamPositions(:, 1), droneSlamPositions(:, 2), droneSlamPositions(:, 3), 'b-', 'LineWidth', 2, 'DisplayName', 'SLAM Position');
end

% Tracement des points slams, si disponible
if ~isempty(xyzPoints)
    scatter3(-xyzPoints(:, 1), -xyzPoints(:, 2), xyzPoints(:, 3), 'b.', 'DisplayName', 'SLAM Points');
end

%plot3(droneGyroposition(:, 1), droneGyroposition(:, 2), droneGyroposition(:, 3), 'o-', 'LineWidth', 2, 'DisplayName', 'Gyro Position');

%% Tracement fictif test
% % Tracement de la position groundtruthopti, si disponible
% if exist('displacement', 'var')
%     plot3(displacement(:, 1), displacement(:, 2), displacement(:, 3), 'r-', 'LineWidth', 2, 'DisplayName', 'SLAM Position');
% end
% 
% if exist('allObjectPositions', 'var')
% %Test tracer des valeurs aléaotire 
% scatter3(allObjectPositions(:, 1), allObjectPositions(:, 2), allObjectPositions(:, 3), 'r.', 'DisplayName', 'objectPositions');
% end


%% Paramètre du graphique

% Personnaliser le tracé
xlabel('X');
ylabel('Y');
zlabel('Z'); %repère optitrack
title('Objets détectés, trajectoire du drone et points SLAM en espace 3D');
grid on;
axis equal;

% Fixer la taille du graphique et centrer l'origine à (0,0,0)
xlim([-4, 4]);
ylim([0, 3]);
zlim([-4, 4]);

% Afficher la légende
%legend('show');

% Démarrer le drone au centre du graphique
view(3)
hold off;


%% Fonction plotcube
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
