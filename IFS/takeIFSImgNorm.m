function [imgIFS, cube] = takeIFSImgNorm(target, DM, camera, DM1command, DM2command)
%% take IFS images, normalize by peak intensity, and adjust the image orientation
% Developed by He Sun on Aug. 30, 2019
% camera - defines the properties of camera

%% Send commands to deformable mirrors
% flip or rotate the DM commands
DM1command2D = zeros(DM.Nact, DM.Nact);
DM2command2D = zeros(DM.Nact, DM.Nact);
DM1command2D(DM.activeActIndex) = DM1command;
DM2command2D(DM.activeActIndex) = DM2command;
DM1command2D = fliplr(DM1command2D);
DM2command2D = rot90(DM2command2D, 2);
DM1command = DM1command2D(DM.activeActIndex);
DM2command = DM2command2D(DM.activeActIndex);
% send commands to DMs
DM1Voltage = DM.DM1bias + DM1command;
DM2Voltage = DM.DM2bias + DM2command; % calculate the true voltage inputs by adding command to flat voltage
voltages = [DM1Voltage; zeros(1024-952, 1); DM2Voltage; zeros(1024-952, 1); zeros(2048, 1)]; % vectorized input to DM driver
BitsCommand = ((2^16-1)/300) * voltages; % convert the command in voltages to bits
BrdNum = 1;
error = BurstHVA4096Frame1D(BrdNum, BitsCommand);

%% take IFS image and extract data cube
[imgIFS, datacube] = takeIFSImg(camera);

%% crop the image and adjust the image orientation
cubeRotate = rotateAndCropIFS(datacube, camera);

%% normalize the cube by peak PSF intensity
cube = zeros(camera.Neta, camera.Nxi, target.broadSampleNum);
for kWavelength = 1 : target.broadSampleNum
    cube(:, :, kWavelength) = cubeRotate(:, :, camera.IFSlamSam(kWavelength)) / (target.fluxBroadband(kWavelength) * camera.exposure);
end

%% flip and rotate the cube
for kWavelength = 1 : target.broadSampleNum
    cube(:, :, kWavelength) = fliplr(cube(:, :, kWavelength));
%     cube(:, :, kWavelength) = flipud(cube(:, :, kWavelength));
%     cube(:, :, kWavelength) = rot90(cube(:, :, kWavelength), 2);
end
end