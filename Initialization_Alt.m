%% Initialize the estimatorAlts
estimatorAlt.type = 'batch';%'ekf_speckle';%'ekf_speckle';%'EKF';%'EKF';%'EKF';%'EKF';%'batch';%'EKF';%'EKF';%'batch';%'batch';%'Kalman';%%'perfect';% % the estimatorAlt type, 'perfect', 'batch', 'Kalman', 'EKF', 'UKF', 'overallKalman', 'preProcessKalman'
estimatorAlt.whichDM = estimator.whichDM;
estimatorAlt.NumImgPair = 1; % Used when EKFpairProbing is 1, if NumImgPair = 1, two images are used (positive and negative versions of the probe command)
estimatorAlt.NumImg = 1; %CHANGE BACK TO 1 % Used when EKFpairProbing is 0
estimatorAlt.linearProbe = 1;%1; % 1 stands for only considering the linear part of DM probing, 0 stands for simulating the probing which include all the terms
estimatorAlt.nonProbeImage = 0;
estimatorAlt.EKFpairProbing = 1; % 1 stands for still using pair-wise probing, 0 stands for not
estimatorAlt.EKFincoherent = 0; % 1 stands for estimating incoherent in EKF, 0 stands for assuming no incoherent light
estimatorAlt.optimized_probe = 0;
estimatorAlt.itrEKF = 10;%10;%10;%3; % Used for 'EKF' only, IEKF iterations to make more accurate estimation
estimatorAlt.itrUKF = 10;%10; % Used for 'UKF' only, which has similar formula to IEKF
estimatorAlt.probeArea = [1, 17, -17, 17]; %[0, 17, -17, 17]; % Define the region in lambda / D
estimatorAlt.probeMethod = 'Empirical'; %'OptimalOffsets';% 'Empirical' or 'OptimalOffsets', choose the best probing offset to reduce state covariance
estimatorAlt.measuredAmp = 0; % 1 or 0, 1 stands for that we adjust the probing amplitude using measured images
estimatorAlt.savedata = 1; % 1 or 0, 1 stands for that we want to save the probing command and images for future run
estimatorAlt.stateStd0 = 2e-7;%7e-6;%1e-6 % the coefficient used to initialize the state covariance, used for Kalman filter and extended Kalman filter
estimatorAlt.processVarCoefficient = 5e-9;%SFR%5e-9;%sfr %5e-9;%6e-9;%3e-8;% 3e-9 for physics model;%3e-8;%0.05 * 1e-7;%0.05 * 1e-7;%0.01 * 1e-7 for EKF 2 pair and UKF 2 images%0.01 * 1e-8; for EKF 1 pair and 1 image%0.3 * 1e-7 for lab% the coefficient used for compute the process covariance noise, used for Kalman filter and extended Kalman filter
estimatorAlt.processVarCoefficient2 = 1e-9;%2e-10;%1e-9;%sfr 1e-7;%1e-9;%sfr %1e-7;% 1e-9;%
estimatorAlt.observationVarCoefficient = 1e-15;%5e-17;%1e-15;%1e-14;%SFR %1e-15;%5e-17;% 1e-14;%3e-14;%1e-16;%3e-14;%3e-15;%1e-14;%6e-18;%6e-18; % the coefficient used for compute the observation covariance noise matrix
estimatorAlt.observationVarCoefficient1 = 1.0 / (target.flux * camera.exposure); % scaling coefficient for camera possion noises
estimatorAlt.observationVarCoefficient2 = 0.8e-14;% 2e-14 for physcis model;%3.68e-13;%5e-14;%7e-13;
estimatorAlt.observationVarCoefficient3 = 0.0015;%0.022;%0.009;%0.022;% for SPC, 0.009;% for SPLC 0.008;% for SPC aberrated
estimatorAlt.observationVarCoefficient10 = 1.0 / (target.flux);
estimatorAlt.observationVarCoefficient0 = 5e-17; % the readout noise parameter normalized by time
% estimatorAlt.processVarCoefficient = 5.16e-9;%3e-9;%3e-8;%3e-8;%0.05 * 1e-7;%0.05 * 1e-7;%0.01 * 1e-7 for EKF 2 pair and UKF 2 images%0.01 * 1e-8; for EKF 1 pair and 1 image%0.3 * 1e-7 for lab% the coefficient used for compute the process covariance noise, used for Kalman filter and extended Kalman filter
% estimatorAlt.processVarCoefficient2 = 1.4e-9;%1e-10;%6.6943e-10;
% estimatorAlt.observationVarCoefficient = 1e-14;%3e-14;%1e-16;%3e-14;%3e-15;%1e-14;%6e-18;%6e-18; % the coefficient used for compute the observation covariance noise matrix
% estimatorAlt.observationVarCoefficient2 = 3.68e-13;%5e-14;%7e-13;
estimatorAlt.incoherentStd0 = 1e-13;%0.1e-7; % the std of incoherent process noise used for 'EKF'
estimatorAlt.incoherentStd = 1e-9;%1e-10;
estimatorAlt.adaptive_exposure = camera.adaptive_exposure;
estimatorAltBatch = estimatorAlt;
estimatorAltBatch.type = 'batch';
estimatorAltBatch.whichDM = '1';
if strcmpi(coronagraph.type, 'VORTEX')
    estimatorAltBatch.NumImgPair = 4;
else
    estimatorAltBatch.NumImgPair = 2;
end
estimatorAltBatch.EKFpairProbing = 1;
estimatorAltBatch.adaptive_exposure = 0;
estimatorAltBatch.activeSensing = 0;
% active sensing part
estimatorAlt.activeSensing = 0;
if estimatorAlt.activeSensing
    cd(folder.python)
    mod = py.importlib.import_module('sensing');
    cd(folder.main)
    estimatorAlt.Q1 = sqrt(estimatorAlt.processVarCoefficient);%1e-6;%
    estimatorAlt.Q3 = sqrt(estimatorAlt.processVarCoefficient2);%1e-7;%
    estimatorAlt.Q4 = 0.0;
    estimatorAlt.Q5 = 1e-9;%1e-10;
    estimatorAlt.R1 = sqrt(estimatorAlt.observationVarCoefficient2);%1e-8;%1e-7;%1e-8;%1e-10;%
    estimatorAlt.R3 = sqrt(estimatorAlt.observationVarCoefficient);%1e-10;%
    estimatorAlt.beta = 1;%1;%3e-1;%1e-2;%952 * 0.7e-1;
    estimatorAlt.rate = 1e-3;%1e-3;%5e-2;%5e-2; %5e-2 for probe scale optimization, 1e1 for probe command optimization
    estimatorAlt.sgd_itr = 200;
end

%% Initialize the structure for saving dataAlt
%-------------------------- Sepckle Nulling -------------------------------
if strcmpi(controller.type, 'speckleNulling') || strcmpi(controller.type, 'MultiSpeckleNulling')
    dataAlt.controllerType = controller.type;
    if target.broadBandControl
        dataAlt.measuredContrastAverage = zeros(Nitr, 1); % the measured average contrast in the dark holes (after wavefront correction)
        dataAlt.measuredContrastMax = zeros(Nitr, 1); % the measured max contrast in the dark holes (after wavefront correction)
        dataAlt.measuredContrastStd = zeros(Nitr, 1); % the std of measured contrast in the dark holes (after wavefront correction)
        dataAlt.DMcommand = zeros(2 * DM.activeActNum, Nitr); % the DM control command for each iteration
        dataAlt.backgroundAverage = zeros(Nitr, 1); % the average contrast of background
        dataAlt.backgroundStd = zeros(Nitr, 1); % the std deviation of the backgroud
        dataAlt.probeImage = zeros(camera.Neta, camera.Nxi, target.broadSampleNum, 4, Nitr); % probe images during speckle nulling control
        dataAlt.I = zeros(camera.Neta, camera.Nxi,target.broadSampleNum, Nitr); % used to save the focal images after each control iteration
    else
        dataAlt.measuredContrastAverage = zeros(Nitr, 1); % the measured average contrast in the dark holes (after wavefront correction)
        dataAlt.measuredContrastMax = zeros(Nitr, 1); % the measured max contrast in the dark holes (after wavefront correction)
        dataAlt.measuredContrastStd = zeros(Nitr, 1); % the std of measured contrast in the dark holes (after wavefront correction)
        dataAlt.DMcommand = zeros(2 * DM.activeActNum, Nitr); % the DM control command for each iteration
        dataAlt.backgroundAverage = zeros(Nitr, 1); % the average contrast of background
        dataAlt.backgroundStd = zeros(Nitr, 1); % the std deviation of the backgroud
        dataAlt.probeImage = zeros(camera.Neta, camera.Nxi, 4, Nitr); % probe images during speckle nulling control
        dataAlt.I = zeros(camera.Neta, camera.Nxi, Nitr); % used to save the focal images after each control iteration
    end
elseif strcmpi(controller.type, 'EFC')
%-------------------------------- EFC -------------------------------------
    dataAlt.controllerType = [controller.type, estimatorAlt.type];
    dataAlt.itr = 0;
    if target.broadBandControl
        dataAlt.I0 = zeros(camera.Neta, camera.Nxi, target.broadSampleNum); % used to save the original focal images
        dataAlt.I = zeros(camera.Neta, camera.Nxi, target.broadSampleNum, Nitr); % used to save the focal images after each control iteration
        dataAlt.EfocalEst = zeros(darkHole.pixelNum, target.broadSampleNum, Nitr); % the estimated coherent electric field at each iteration
        dataAlt.IincoEst = zeros(darkHole.pixelNum, target.broadSampleNum, Nitr); % the estimated incoherent light itensity
        dataAlt.EfocalEstPerfect = zeros(darkHole.pixelNum, target.broadSampleNum, Nitr); % the estimated coherent electric field at each iteration
        dataAlt.IincoEstPerfect = zeros(darkHole.pixelNum, target.broadSampleNum, Nitr); % the estimated incoherent light itensity
        dataAlt.measuredContrastAverage = zeros(target.broadSampleNum, Nitr); % the measured average contrast in the dark holes (after wavefront correction)
        dataAlt.estimatedContrastAverage = zeros(target.broadSampleNum, Nitr); % the estimated average contrast in the dark holes (before wavefront correction)
        dataAlt.estimatedIncoherentAverage = zeros(target.broadSampleNum, Nitr); % the estimated average incoherent contrast in the dark holes (before wavefront correction)
        dataAlt.measuredContrastMax = zeros(target.broadSampleNum, Nitr); % the measured max contrast in the dark holes (after wavefront correction)
        dataAlt.estimatedContrastMax = zeros(target.broadSampleNum, Nitr); % the estimated max contrast in the dark holes (before wavefront correction)
        dataAlt.measuredContrastStd = zeros(target.broadSampleNum, Nitr); % the std of measured contrast in the dark holes (after wavefront correction)
        dataAlt.estimatedContrastStd = zeros(target.broadSampleNum, Nitr); % the std of estimated contrast in the dark holes (before wavefront correction)
        dataAlt.DMcommand = zeros(2 * DM.activeActNum, Nitr); % the DM control command for each iteration
        dataAlt.backgroundAverage = zeros(target.broadSampleNum, Nitr); % the average contrast of background
        dataAlt.backgroundStd = zeros(target.broadSampleNum, Nitr); % the std deviation of the backgroud
        dataAlt.probeContrast = zeros(Nitr, 1); % the probe contrast
        dataAlt.y = zeros(darkHole.pixelNum, estimatorAlt.NumImgPair, Nitr); % the difference images
        dataAlt.yBroadband = zeros(darkHole.pixelNum, estimatorAlt.NumImgPair, target.broadSampleNum, Nitr); % the difference images
        switch lower(estimatorAlt.type)
            case {'perfect'}
            case {'kalman', 'batch'}
%                 dataAlt.P = zeros(2, 2, darkHole.pixelNum, Nitr);
%                 dataAlt.Pbroadband = zeros(2, 2, darkHole.pixelNum, target.broadSampleNum, Nitr);
                dataAlt.P = zeros(2, 2, darkHole.pixelNum, target.broadSampleNum, Nitr);
                dataAlt.uProbe = zeros(DM.activeActNum, estimatorAlt.NumImgPair, Nitr); % the probe shapes
            case {'ekf', 'ukf'}
%                 dataAlt.P = zeros(3, 3, darkHole.pixelNum, Nitr);
%                 dataAlt.Pbroadband = zeros(3, 3, darkHole.pixelNum, target.broadSampleNum, Nitr);
                dataAlt.P = zeros(3, 3, darkHole.pixelNum, target.broadSampleNum, Nitr);
                dataAlt.uProbe = zeros(DM.activeActNum, estimatorAlt.NumImg, Nitr); % the probe shapes
            otherwise
                disp('The estimatorAlt type can only be batch, kalman, overallkalman, ekf or ukf');
                return;
        end
        if estimatorAlt.savedata
            dataAlt.imageSet = cell(Nitr, 1); % the image set used for estimation
            dataAlt.probeSet = cell(Nitr, 1); % the command set for probing
        end
        if strcmpi(simOrLab, 'simulation')
            dataAlt.coherentErr = zeros(target.broadSampleNum, Nitr);
            dataAlt.incoherentErr = zeros(target.broadSampleNum, Nitr);
            dataAlt.Efocaltrue = zeros(darkHole.pixelNum,Nitr);
            dataAlt.Iincotrue = zeros(darkHole.pixelNum,Nitr);
        end
        dataAlt.contrast0 = zeros(target.broadSampleNum, 1);
        dataAlt.contrast0Max = zeros(target.broadSampleNum, 1);
        dataAlt.contrast0Std = zeros(target.broadSampleNum, 1);
        dataAlt.estimatedContrastAverage0 = zeros(target.broadSampleNum, 1);
        dataAlt.estimatedContrastMax0 = zeros(target.broadSampleNum, 1);
        dataAlt.estimatedContrastStd0 = zeros(target.broadSampleNum, 1);
        dataAlt.estimatedIncoherentAverage0 = zeros(target.broadSampleNum, 1);
        dataAlt.EfocalEst0 = zeros(darkHole.pixelNum, target.broadSampleNum);
        dataAlt.IincoEst0 = zeros(darkHole.pixelNum, target.broadSampleNum);
    else
        dataAlt.y0 = zeros(darkHole.pixelNum, estimatorAlt.NumImgPair);
        dataAlt.P0 = zeros(2, 2, darkHole.pixelNum);
        dataAlt.I = zeros(camera.Neta, camera.Nxi, Nitr); % used to save the focal images after each control iteration
        dataAlt.EfocalEst = zeros(darkHole.pixelNum, Nitr); % the estimated coherent electric field at each iteration
        dataAlt.EfocalEstProbed = zeros(darkHole.pixelNum, Nitr); % the estimated coherent electric field for probed version to compare with dither method at each iteration
        dataAlt.IincoEst = zeros(darkHole.pixelNum, Nitr); % the estimated incoherent light itensity
        dataAlt.measuredContrastAverage = zeros(Nitr, 1); % the measured average contrast in the dark holes (after wavefront correction)
        dataAlt.estimatedContrastAverage = zeros(Nitr, 1); % the estimated average contrast in the dark holes (before wavefront correction)
        dataAlt.estimatedIncoherentAverage = zeros(Nitr, 1); % the estimated average incoherent contrast in the dark holes (before wavefront correction)
        dataAlt.measuredContrastMax = zeros(Nitr, 1); % the measured max contrast in the dark holes (after wavefront correction)
        dataAlt.estimatedContrastMax = zeros(Nitr, 1); % the estimated max contrast in the dark holes (before wavefront correction)
        dataAlt.measuredContrastStd = zeros(Nitr, 1); % the std of measured contrast in the dark holes (after wavefront correction)
        dataAlt.estimatedContrastStd = zeros(Nitr, 1); % the std of estimated contrast in the dark holes (before wavefront correction)
        dataAlt.estimatedContrastErr = zeros(Nitr, 1);
        dataAlt.DMcommand = zeros(2 * DM.activeActNum, Nitr); % the DM control command for each iteration
        dataAlt.backgroundAverage = zeros(Nitr, 1); % the average contrast of background
        dataAlt.backgroundStd = zeros(Nitr, 1); % the std deviation of the backgroud
        dataAlt.probeContrast = zeros(Nitr, 1); % the probe contrast
        if strcmpi(estimatorAlt.type, 'ekf') || strcmpi(estimatorAlt.type, 'ekf_speckle')% && ~estimatorAlt.EKFpairProbing  SUSAN CHANGED THIS ****************************
            if estimatorAlt.nonProbeImage
                dataAlt.y = zeros(darkHole.pixelNum, estimatorAlt.NumImg+1, Nitr);
            else
                dataAlt.y = zeros(darkHole.pixelNum, estimatorAlt.NumImg, Nitr); % the difference images
            end
        else
            if estimatorAlt.EKFpairProbing
                dataAlt.y = zeros(darkHole.pixelNum, estimatorAlt.NumImgPair, Nitr); % the difference images
            else
                dataAlt.y = zeros(darkHole.pixelNum, estimatorAlt.NumImg, Nitr); % the difference images
            end
        end
        
        switch lower(estimatorAlt.type)
            case {'perfect'}
            case {'kalman', 'batch'}
                dataAlt.P = zeros(2, 2, darkHole.pixelNum, Nitr);
                if estimatorAlt.EKFpairProbing
                    if strcmpi(estimatorAlt.whichDM, 'both')
                        dataAlt.uProbe = zeros(DM.activeActNum*2, estimatorAlt.NumImgPair, Nitr); % the probe shapes
                    else
                        dataAlt.uProbe = zeros(DM.activeActNum, estimatorAlt.NumImgPair, Nitr); % the probe shapes
                    end
                else
                    if strcmpi(estimatorAlt.whichDM, 'both')
                        dataAlt.uProbe = zeros(DM.activeActNum*2, estimatorAlt.NumImg, Nitr); % the probe shapes
                    else
                        dataAlt.uProbe = zeros(DM.activeActNum, estimatorAlt.NumImg, Nitr); % the probe shapes
                    end
                end
            case {'ekf', 'ukf', 'ekf_speckle'} %SUSAN ADDED THIS*******************************************
                if estimatorAlt.EKFincoherent
                    dataAlt.P = zeros(3, 3, darkHole.pixelNum, Nitr);
                else
                    dataAlt.P = zeros(2, 2, darkHole.pixelNum, Nitr);
                end
                if strcmpi(estimatorAlt.whichDM, 'both')
                    dataAlt.uProbe = zeros(DM.activeActNum*2, estimatorAlt.NumImg, Nitr); % the probe shapes
                else
                    dataAlt.uProbe = zeros(DM.activeActNum, estimatorAlt.NumImg, Nitr); % the probe shapes
                end
            otherwise
                disp('The estimatorAlt type can only be batch, kalman, overallkalman, ekf or ukf');
                return;
        end
        if estimatorAlt.savedata
            dataAlt.imageSet = cell(Nitr, 1); % the image set used for estimation
            dataAlt.probeSet = cell(Nitr, 1); % the command set for probing
        end
        if strcmpi(simOrLab, 'simulation')
            dataAlt.coherentErr = zeros(Nitr, 1);
            dataAlt.incoherentErr = zeros(Nitr, 1);
        end
    end
end
dataAlt.estimator = estimatorAlt;
dataAlt.probe_exposure = zeros(Nitr, 1);
if strcmpi(simOrLab, 'simulation')
    dataAlt.EfocalPerfect = zeros(darkHole.pixelNum, Nitr);
end
