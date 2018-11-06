%% main_speckleNulling.m
% the main function to run speckleNulling
% Developed by He Sun on Oct. 2nd
% 
clc;
clear;
close all;

%% Initialize the system and parameters
Nitr = 20;%4000; % iterations of control loop
cRange = [-10, -4]; %[-12, -3];% the range for display
simOrLab = 'simulation'; % 'simulation' or 'lab', run the wavefront correction loops in simulation or in lab
runTrial = 1;
Initialization;

%% ----------------------- Speckle Nulling Algorithm ----------------------
% take focal plane image with no DM poking
camera.exposure = 1;
DM1command = zeros(DM.activeActNum, 1);
DM2command = zeros(DM.activeActNum, 1);

if target.broadBandControl
    I0 = zeros(camera.Neta, camera.Nxi);
    for k = 1 : target.broadSampleNum
        targetmon = target;
        targetmon.starWavelength = target.starWavelengthBroad(k);
        Imon = getImg(targetmon, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
        I0(:, :, k) = Imon;
    end
else
    I0 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
end

if target.broadBandControl
    Imean = mean(I0, 3);
    contrast0 = mean(Imean(darkHole.pixelIndex));
    contrast0Max = max(Imean(darkHole.pixelIndex));
    contrast0Std = std(Imean(darkHole.pixelIndex));
else
    contrast0 = mean(I0(darkHole.pixelIndex));
    contrast0Max = max(I0(darkHole.pixelIndex));
    contrast0Std = std(I0(darkHole.pixelIndex));
end
data.contrast0 = contrast0;
data.contrast0Max = contrast0Max;
data.contrast0Std = contrast0Std;
disp('***********************************************************************');
disp(['The starting measured average contrast in the dark holes is ', num2str(contrast0)]);
disp('***********************************************************************');
if target.broadBandControl
    figure(1), imagesc(log10(abs(Imean))), colorbar;
else
    figure(1), imagesc(log10(abs(I0))), colorbar;
end
caxis(cRange);
drawnow

% Control loop starts
I = I0;
for itr = 1 : Nitr
    %% print the information of current iteration
    disp('***********************************************************************');
    disp(['Now we are running iteration ', num2str(itr) ,'/', num2str(Nitr)]);
    disp('***********************************************************************');
    % crop the center blocked region of the image, calculate the background
    % average contrast and std
    if target.broadBandControl
        background = mean(I(camera.blockedCoordEta, camera.blockedCoordXi, :), 3);
    else
        background = I(camera.blockedCoordEta, camera.blockedCoordXi);
    end
    backgroundAverage = mean(background(:));
    backgroundStd = std(background(:));
    data.backgroundAverage(itr) = backgroundAverage;
    data.backgroundStd(itr) = backgroundStd;
    
    % find the DM command via speckle nulling
    if strcmpi(controller.type, 'MultiSpeckleNulling')
        [command, probeImage] = MultiSpeckleNulling(I, target, DM, coronagraph, camera, controller, darkHole, DM1command, DM2command, simOrLab);
    elseif strcmpi(controller.type, 'speckleNulling')
        [command, probeImage] = SpeckleNulling(I, target, DM, coronagraph, camera, controller, darkHole, DM1command, DM2command, simOrLab);
    end
    switch controller.whichDM
        case '1'
            DM1command = DM1command + command;
        case '2'
            DM2command = DM2command + command;
        otherwise
            disp('You can only use the first DM or the second DM for speckle nulling wavefront control.');
            return;
    end
    if target.broadBandControl
        I = zeros(camera.Neta, camera.Nxi);
        for k = 1 : target.broadSampleNum
            targetmon = target;
            targetmon.starWavelength = target.starWavelengthBroad(k);
            targetmon.normalization = target.normalizationBroadband(k);
            Imon = getImg(targetmon, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
            I(:, :, k) = Imon;
        end
    else
        I = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
    end
    
    % save the data in control loop
    if target.broadBandControl
        Imean = mean(I, 3);
        data.measuredContrastAverage(itr) = mean(Imean(darkHole.pixelIndex));
        data.measuredContrastMax(itr) = max(Imean(darkHole.pixelIndex)); 
        data.measuredContrastStd(itr) = std(Imean(darkHole.pixelIndex));
        data.DMcommand(:, itr) = [DM1command; DM2command];
        data.I(:, :, :, itr) = I;
        data.probeImage(:, :, :, :, itr) = probeImage;
    else
        data.measuredContrastAverage(itr) = mean(I(darkHole.pixelIndex));
        data.measuredContrastMax(itr) = max(I(darkHole.pixelIndex)); 
        data.measuredContrastStd(itr) = std(I(darkHole.pixelIndex));
        data.DMcommand(:, itr) = [DM1command; DM2command];
        data.I(:, :, itr) = I;
        data.probeImage(:, :, :, itr) = probeImage;
    end
    % visualize the camera measurements and control curves
    if target.broadBandControl
        figure(1), imagesc(log10(abs(Imean))), colorbar;
        caxis(cRange);
        drawnow
    else
        figure(1), imagesc(log10(abs(I))), colorbar;
        caxis(cRange);
        drawnow
    end
    figure(2), semilogy(0:itr, [contrast0; data.measuredContrastAverage(1:itr)]);
    ylim([10^(cRange(1)), 10^(cRange(2))]);
    drawnow
    % print results
    disp(['The average contrast of the background light is ', num2str(backgroundAverage)]);
    disp(['The std of the background light is (camera noises) ', num2str(backgroundStd)]);
    disp(['The measured average contrast in the dark holes is ', num2str(data.measuredContrastAverage(itr))]);
    if data.measuredContrastAverage(itr) < 1e-5
        camera.exposure = 1;
    end
end

%% ----------------------- EFC Algorithm ----------------------

%% save data
eval([data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), '=data;']);
cd(folder.dataLibrary);
eval(['save ', data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), ' ', data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), ';']);
cd(folder.main);