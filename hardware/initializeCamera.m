function camera = initializeCamera(camera)
%% initialize the camera drivers
% Developed by He Sun on Feb. 23, 2017
%
% camera - defines the properties of the camera
%
camera.handle = Camera_ctrl(0, 'enable'); % connect the computer to camera, save the handle
Camera_ctrl(camera.handle, 'init', -15); % enable the camera, set up the camera temperature
Camera_ctrl(camera.handle, 'shutterpriority', 1);
camera.handle = Camera_ctrl(camera.handle, 'shutter', 1); % open the camera shutter

Camera_ctrl(camera.handle, 'exposureproperties', camera.startPosition, camera.imageSize,...
            [camera.binXi, camera.binEta]); % set up camera properties
if(camera.newDarkFrame) % take new dark frame if needed
    numIm = 30;
    camera.darkFrame = takeDarkCam(camera.handle, camera.exposure, numIm, camera.binXi, camera.binEta);
else
    temp = load('darkCam.mat');
    camera.darkFrame = temp.darkCam;
%     camera.darkFrame = zeros(camera.imageSize);
end
end