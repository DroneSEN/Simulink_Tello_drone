% clear Tello

% Tello = ryze("Tello");
% TelloCamera = camera(Tello);
% ImageFolder ='/Bureau/9BA074/';
% 
% 
% for k=1:30
%     Image = snapshot(TelloCamera);
%     file_name = sprintf('Image%d.png',k);
%     imgName = fullfile(ImageFolder,file_name) ;
%     imwrite(Image,imgName);
%     imshow(Image);
%     pause(1);
% 
% end
% 
% clear;

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
