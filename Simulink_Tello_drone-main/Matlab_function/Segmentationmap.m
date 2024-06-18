% Charger les données depuis le workspace
objectXYZrotmatrix = evalin('base', 'objectXYZrotmatrix');
objectXYZangleteta = evalin('base', 'objectXYZangleteta');
distances = evalin('base', 'distances');
objectDimensions = load('Matlab_System/objectDimensions.mat');
targetObject = 'tvmonitor'; % Spécifiez ici l'objet cible

% Récupérer les dimensions moyennes de l'objet
dimensions = objectDimensions.objectDimensions.(targetObject);
realHeight = dimensions(1);
realWidth = dimensions(2);
realDepth = dimensions(3);

% Tracer la carte
figure;
hold on;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Object Detection Map');

% Tracer les points des objets détectés avec la matrice de rotation
scatter3(objectXYZrotmatrix(:, 1), objectXYZrotmatrix(:, 2), objectXYZrotmatrix(:, 3), 'filled');
text(objectXYZrotmatrix(:, 1), objectXYZrotmatrix(:, 2), objectXYZrotmatrix(:, 3), 'rotmatrix');

% Tracer les points des objets détectés avec les angles theta
scatter3(objectXYZangleteta(:, 1), objectXYZangleteta(:, 2), objectXYZangleteta(:, 3), 'filled');
text(objectXYZangleteta(:, 1), objectXYZangleteta(:, 2), objectXYZangleteta(:, 3), 'angleteta');

% Dessiner des boîtes englobantes pour les objets détectés
for i = 1:size(objectXYZrotmatrix, 1)
    % Définir les coins de la boîte englobante
    x = objectXYZrotmatrix(i, 1);
    y = objectXYZrotmatrix(i, 2);
    z = objectXYZrotmatrix(i, 3);
    corners = [x, y, z;
               x + realWidth, y, z;
               x + realWidth, y + realHeight, z;
               x, y + realHeight, z;
               x, y, z + realDepth;
               x + realWidth, y, z + realDepth;
               x + realWidth, y + realHeight, z + realDepth;
               x, y + realHeight, z + realDepth];
    
    % Tracer les arêtes de la boîte englobante
    for j = 1:4
        k = mod(j, 4) + 1;
        plot3([corners(j, 1), corners(k, 1)], [corners(j, 2), corners(k, 2)], [corners(j, 3), corners(k, 3)], 'r-');
        plot3([corners(j + 4, 1), corners(k + 4, 1)], [corners(j + 4, 2), corners(k + 4, 2)], [corners(j + 4, 3), corners(k + 4, 3)], 'r-');
        plot3([corners(j, 1), corners(j + 4, 1)], [corners(j, 2), corners(j + 4, 2)], [corners(j, 3), corners(j + 4, 3)], 'r-');
    end
end

hold off;
