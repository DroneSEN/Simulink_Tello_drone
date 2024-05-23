% Effacer les objets Tello et TelloCamera s'ils existent déjà
clear Tello
clear TelloCamera

% Créer l'objet Tello et se connecter au drone
Tello = ryze("Tello");
% 
% % % Activer les mission pads
%activateMissionPad(Tello);
% pause(2)
deactivateMissionPad(Tello);

% Activer les deux caméras (avant et arrière)
switch_camera(Tello,"front");
TelloCamera = camera(Tello);

% Prévisualiser la caméra Tello
preview(TelloCamera);
