clear;

% =========================================================================
% Paramètres de calibration
% =========================================================================
% Sélection de la direction de la caméra (Forward ou Downward)
camera_dir = VideoDirection.Downward;

% Nom du drone à calibrer
DroneName = '9BA0DC';


% =========================================================================
% Procédure de calibration
% =========================================================================
Tello = ryze("Tello");
TelloCamera = camera(Tello);

% Détermination du répertoire


ImageFolder = fullfile(pwd(), 'camera_calibration', strcat('Camera_calibration_', DroneName));
if camera_dir == VideoDirection.Forward
    ImageFolder = fullfile(ImageFolder,'front');
else
    ImageFolder = fullfile(ImageFolder,'down');
end

% Switch vers la bonne caméra
switch_camera(Tello, camera_dir);

% cam = webcam(1);
preview(TelloCamera)


answer = questdlg('Start calibration ?', 'Yes', 'Cancel');

% Start calibration
if strcmp(answer,'Yes')
    for k=1:30
        Image = snapshot(TelloCamera);

        if camera_dir == VideoDirection.Downward
            Image = flip(transpose(Image(:,:,1)), 2);
        end

        file_name = sprintf('Image%d.png',k);
        imgName = fullfile(ImageFolder,file_name);
        if ~isempty(Image)
            imwrite(Image,imgName);
            imshow(Image);
        else
            warning("An error occured with image n°%d, not saved", k);
        end
        pause(1);
    end
end

% Reset camera
switch_camera(Tello, VideoDirection.Forward);

clear;

%run cameraCalibrator size square = 34, 6X8

% % Spécifiez le dossier contenant les images
% input_folder = 'C:\Users\user\OneDrive - ESTACA\Documents\DRONE_SEN4\Simulink_Tello_drone-main\camera_calibration\Camera_calibration_img'; % Remplacez par le chemin de votre dossier
% output_folder = fullfile(input_folder, 'resized_images');
% 
% % Créez le dossier de sortie s'il n'existe pas
% if ~exist(output_folder, 'dir')
%     mkdir(output_folder);
% end
% 
% % Liste de tous les fichiers dans le dossier d'entrée
% image_files = dir(fullfile(input_folder, '*.*'));
% image_files = image_files(~[image_files.isdir]); % Exclure les dossiers
% 
% % Dimensions de redimensionnement
% new_width = 480;
% new_height = 360;
% 
% % Parcourir chaque fichier image
% for i = 1:length(image_files)
%     % Nom et chemin complet de l'image actuelle
%     image_name = image_files(i).name;
%     image_path = fullfile(input_folder, image_name);
% 
%     % Lire l'image
%     try
%         img = imread(image_path);
% 
%         % Redimensionner l'image
%         resized_img = imresize(img, [new_height, new_width]);
% 
%         % Chemin de sortie de l'image redimensionnée
%         output_path = fullfile(output_folder, image_name);
% 
%         % Enregistrer l'image redimensionnée
%         imwrite(resized_img, output_path);
%     end
% 
% end
% 
% disp('Redimensionnement terminé pour toutes les images.');
