function [] = finalizeCamera(camera)
%% Finalize the camera - close the mechanical shutter, delete the camera handle
% Developed by He Sun on Feb. 24, revised from He Sun's
% "Lab_initialization.m" file
%
% camera - defines the properties of the camera
%
camera.handle = Camera_ctrl(camera.handle, 'shutter', 0); % close the mechanical shutter
Camera_ctrl(camera.handle, 'finalize'); % fan off, cooler off and shutter off
Camera_ctrl(camera.handle, 'disable'); % disconnect the camera from the computer
disp('Camera disconnected.')
end
