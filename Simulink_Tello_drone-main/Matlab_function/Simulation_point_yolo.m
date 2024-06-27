% Nombre de points à générer
numPoints = 1000;  % Modifier selon le besoin

% Échelle de l'erreur (écart-type du bruit)
errorScalescatter = 0.1;
errorScalesgroundtruth = 0.001;

%% Simulation scatter objet estimé 
% Positions initiales des objets
posobj1 = [-0.8 1 2];
posobj2 = [0.8 1 2];

% Génération des données pour l'objet 1
objectPositions1 = repmat(posobj1, numPoints, 1) + errorScalescatter * randn(numPoints, 3);

% Génération des données pour l'objet 2
objectPositions2 = repmat(posobj2, numPoints, 1) + errorScalescatter * randn(numPoints, 3);
% Concaténation des deux arrays
allObjectPositions = [objectPositions1; objectPositions2];

%% Simulation déplacement sur X

% Génération des données de déplacement
displacement = [0,0,0 ; 0, 1, 0; 1, 1, 0; 1, 1 ,1; 0 ,1 , 1; 0,1,0];
