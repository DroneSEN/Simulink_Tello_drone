clear Tello

Tello = ryze("Tello");
TelloCamera = camera(Tello);
ImageFolder ='/Bureau/9BA074/';


for k=1:30
    Image = snapshot(TelloCamera);
    file_name = sprintf('Image%d.png',k);
    imgName = fullfile(ImageFolder,file_name) ;
    imwrite(Image,imgName);
    imshow(Image);
    pause(1);
    
end

clear;

%run cameraCalibrator size square = 34, 6X8