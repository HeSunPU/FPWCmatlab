function [img, datacube] = takeIFSImg(camera)
%% take IFS image and extract the data cube using Crispy
% Developed by He Sun on Aug. 30, 2019
% camera - defines the properties of camera

%% take an image using the Starlight Express camera
[img,x,y,SAT]=SXcam_Nimg(camera.handle, camera.darkFrame, 1000 * camera.exposure, ...
                       camera.Xc, camera.Yc, camera.RX, camera.RY, ...
                       0, 0, camera.stacking);
%     img = imresize(img, [1024, 1024]);
img = img(:, 57:1136-56); % crop the image to 1024x1024
%% extract the data cube using Crispy Python interface
if exist('C:\Lab\FPWCmatlab\IFS\flags\crispyflag.txt')==2
    datacube = extractCube(img);
else
    disp('CRISPY is not running!!! Please initialize CRISPY first while using IFS!!!')
end


end