% Nombre de points à générer
numPoints = 1000;  % Modifier selon le besoin

% Échelle de l'erreur (écart-type du bruit)
errorScalescatter = 0.1;
errorScalesgroundtruth = 0.001;

%% Simulation scatter objet estimé 
% Positions initiales des objets
posobj1 = [-1 1 2];
posobj2 = [1 1 2];

% Génération des données pour l'objet 1
objectPositions1 = repmat(posobj1, numPoints, 1) + errorScalescatter * randn(numPoints, 3);

% Génération des données pour l'objet 2
objectPositions2 = repmat(posobj2, numPoints, 1) + errorScalescatter * randn(numPoints, 3);
% Concaténation des deux arrays
allObjectPositions = [objectPositions1; objectPositions2];

%% Simulation déplacement sur X
% Déplacement le long de l'axe X, de manière aléatoire entre -1 et 1
xDisplacement = -1 + 2 * rand(numPoints, 1);  % Génère aléatoirement des valeurs entre -1 et 1

% Y est constant à 1 mais avec un léger bruit
yWithNoise = 1 + errorScalesgroundtruth * randn(numPoints, 1);

% Z est constant à 0 mais avec un léger bruit
zWithNoise = errorScalesgroundtruth * randn(numPoints, 1);

% Génération des données de déplacement
displacement = [xDisplacement, yWithNoise, zWithNoise];
