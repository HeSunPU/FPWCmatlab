
clc;
clear;
close all;

%% Initialize the system and parameters
Nitr =5;%4000; % iterations of control loop
cRange = [-8, -3]; %[-12, -3];% the range for display
simOrLab ='simulation';%   'lab';%'simulation' or 'lab', run the wavefront correction loops in simulation or in lab
runTrial = 1;
Initialization;

%% Initialize the hardware driver if we are running experiment
% Laser_Enable('on');
% Laser_Power(65, 1);
%% 
if (strcmpi(simOrLab, 'lab')) % if conducting experiment in lab, initialize DM and camera drivers
    camera = initializeCamera(camera);
    
%     DM.DM1bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_CLOSED_LOOP_200nm_VOLTAGES.txt','-ascii');
    DM.DM1bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW004#14_CLOSED_LOOP_200nm_Voltages_DM#1.txt','-ascii');
    DM.DM1bias = DM.DM1bias(1 : DM.activeActNum); % flatten map voltages in volts on DM1
%     DM.DM2bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#014_CLOSED_LOOP_200nm_Voltages.txt','-ascii');
    DM.DM2bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW018#40_CLOSED_LOOP_200nm_Voltages_DM#2.txt','-ascii');
    DM.DM2bias = DM.DM2bias(1 : DM.activeActNum); % flatten map voltages in volts on DM2
    initializeDM(DM);
%     camera = initializeCamera(camera);
end
%% disconnect the camera at the end
disconnecting = 0;
if disconnecting == 1
	error = BurstHVA4096Frame1D(1, zeros(4096,1)); % finalize DMs
	finalizeCamera(camera) % finalize the camera
    Laser_Power(0, target.starChannel)
    Laser_Enable('off')    
end
%% Compute the state space model of the system
% parpool(16);
if target.broadBandControl
%     G1Broadband = zeros(darkHole.pixelNum, DM.activeActNum, target.broadSampleNum);
%     G2Broadband = zeros(darkHole.pixelNum, DM.activeActNum, target.broadSampleNum);
%     for kWavelength = 1 : target.broadSampleNum
%         target.starWavelength = target.starWavelengthBroad(kWavelength);
%         model = stateSpace(target, DM, coronagraph, camera, darkHole);
%         G1Broadband(:, :, kWavelength) = model.G1;
%         G2Broadband(:, :, kWavelength) = model.G2;
%     end
%     model.G1 = G1Broadband;
%     model.G2 = G2Broadband;
    load modelBroadband.mat
    model = modelBroadband;
%     load model4.mat
else
    model = stateSpace(target, DM, coronagraph, camera, darkHole);
%     load model.mat
end

%% define active sensor if required
if estimator.activeSensing 
    G1real = py.numpy.array(real(model.G1));
    G1imag = py.numpy.array(imag(model.G1));
    G2real = py.numpy.array(real(model.G2));
    G2imag = py.numpy.array(imag(model.G2));
    switch lower(estimator.type)
        case 'batch'
            if estimator.EKFpairProbing
                estimator.sensor = py.sensing.active_sensing11(G1real, G1imag, G2real, G2imag, estimator.NumImgPair, ...
                                                            estimator.observationVarCoefficient0, estimator.observationVarCoefficient10, estimator.observationVarCoefficient3);
            end
        case 'kalman'
            if estimator.EKFpairProbing
%                 estimator.sensor = py.sensing.active_sensing5(G1real, G1imag, G2real, G2imag, estimator.NumImgPair, ...
%                                                             estimator.processVarCoefficient, estimator.processVarCoefficient2, ...
%                                                             estimator.observationVarCoefficient0, estimator.observationVarCoefficient10, estimator.observationVarCoefficient3);
%                 estimator.sensor = py.sensing.active_sensing6(G1real, G1imag, G2real, G2imag, estimator.NumImgPair, ...
%                                                             estimator.processVarCoefficient, estimator.processVarCoefficient2, ...
%                                                             estimator.observationVarCoefficient0, estimator.observationVarCoefficient10, estimator.observationVarCoefficient3);
    %             estimator.sensor = py.sensing.active_sensing7(G1real, G1imag, G2real, G2imag, estimator.NumImgPair, ...
%                                                         estimator.processVarCoefficient, estimator.processVarCoefficient2, ...
%                                                         estimator.observationVarCoefficient0, estimator.observationVarCoefficient10, estimator.observationVarCoefficient3);
                estimator.sensor = py.sensing.active_sensing12(G1real, G1imag, G2real, G2imag, estimator.NumImgPair, ...
                                                            estimator.processVarCoefficient, estimator.processVarCoefficient2, ...
                                                            estimator.observationVarCoefficient0, estimator.observationVarCoefficient10, estimator.observationVarCoefficient3);
            end
        case 'ekf'
            if ~estimator.EKFpairProbing
                if strcmpi(estimator.whichDM, 'both')
%                     estimator.sensor = py.sensing.active_sensing10(G1real, G1imag, G2real, G2imag, estimator.NumImg, ...
%                                                                 estimator.processVarCoefficient, estimator.processVarCoefficient2, ...
%                                                                 estimator.observationVarCoefficient0, estimator.observationVarCoefficient10, estimator.observationVarCoefficient3);
                    estimator.sensor = py.sensing.active_sensing16(G1real, G1imag, G2real, G2imag, estimator.NumImg, ...
                                                                estimator.processVarCoefficient, estimator.processVarCoefficient2, ...
                                                                estimator.observationVarCoefficient0, estimator.observationVarCoefficient10, estimator.observationVarCoefficient3);
                else
%                     estimator.sensor = py.sensing.active_sensing8(G1real, G1imag, G2real, G2imag, estimator.NumImg, ...
%                                                                 estimator.processVarCoefficient, estimator.processVarCoefficient2, ...
%                                                                 estimator.observationVarCoefficient0, estimator.observationVarCoefficient10, estimator.observationVarCoefficient3);
%                     estimator.sensor = py.sensing.active_sensing9(G1real, G1imag, G2real, G2imag, estimator.NumImg, ...
%                                                                 estimator.processVarCoefficient, estimator.processVarCoefficient2, ...
%                                                                 estimator.observationVarCoefficient0, estimator.observationVarCoefficient10, estimator.observationVarCoefficient3);
                    estimator.sensor = py.sensing.active_sensing14(G1real, G1imag, G2real, G2imag, estimator.NumImg, ...
                                                                estimator.processVarCoefficient, estimator.processVarCoefficient2, ...
                                                                estimator.observationVarCoefficient0, estimator.observationVarCoefficient10, estimator.observationVarCoefficient3);
                end
            end
        otherwise
            disp('Not using active sensing!')
    end
end
%%
% for kCorrection = 1 : 10
%     runTrial = kCorrection;
%% take focal plane image with no DM poking
% For  thorlabs laser:
camera.exposure = 0.05;%0.01;
camera.exposure0 = 0.05;%0.01;
% For superK
% camera.exposure = 0.1;
% camera.exposure0 = 0.1;

DM1command = zeros(DM.activeActNum, 1);
DM2command = zeros(DM.activeActNum, 1);
if target.broadBandControl
    target_help = target;
    for kWavelength = 1 : target.broadSampleNum
        target_help.starWavelegth = target.starWavelengthBroad(kWavelength);
        I0 = getImg(target_help, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
        contrast0 = mean(I0(darkHole.pixelIndex));
        contrast0Max = max(I0(darkHole.pixelIndex));
        contrast0Std = std(I0(darkHole.pixelIndex));
        data.contrast0(kWavelength) = contrast0;
        data.contrast0Max(kWavelength) = contrast0Max;
        data.contrast0Std(kWavelength) = contrast0Std;
        data.I0(:, :, kWavelength) = I0;
    end
    %% estimate the starting contrast using batch process estimation
    contrastEst = -1;
    [imageAll, u, data] = takeProbingImagesBroad(contrastEst, target, DM, coronagraph, camera, darkHole, estimatorBatch, DM1command, DM2command, simOrLab, data);
    for kWavelength = 1 : target.broadSampleNum
        model_help.G1 = squeeze(model.G1(:, :, kWavelength));
        model_help.G2 = squeeze(model.G2(:, :, kWavelength));
        image = squeeze(imageAll(:, :, :, kWavelength));
        [EfocalEst, IincoEst, data] = batch(u, image, darkHole, model_help, estimatorBatch, data);
        EfocalEst(abs(EfocalEst).^2 > 1e-2) = 0;
        data.EfocalEst0(:, kWavelength) = EfocalEst;
        data.IincoEst0(:, kWavelength) = IincoEst;
        IfocalEst = abs(EfocalEst).^2;
        data.estimatedContrastAverage0(kWavelength) = mean(IfocalEst);
        data.estimatedContrastMax0(kWavelength) = max(IfocalEst);
        data.estimatedContrastStd0(kWavelength) = std(IfocalEst);
        data.estimatedIncoherentAverage0(kWavelength) = mean(IincoEst);
    end
    contrastEst = mean(data.estimatedContrastAverage0);
    incoherentEst = mean(data.estimatedIncoherentAverage0);
    EfocalEstBroadband = data.EfocalEst0;
    IincoEstBroadband = data.IincoEst0;
else
    I0 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
    contrast0 = mean(I0(darkHole.pixelIndex));
    contrast0Max = max(I0(darkHole.pixelIndex));
    contrast0Std = std(I0(darkHole.pixelIndex));
    data.I0 = I0;
    data.contrast0 = contrast0;
    data.contrast0Max = contrast0Max;
    data.contrast0Std = contrast0Std;
    % estimate the starting contrast using batch process estimation
    contrastEst = -1;
    [image, u, data] = takeProbingImages(contrastEst, target, DM, coronagraph, camera, darkHole, estimatorBatch, DM1command, DM2command, simOrLab, data);
    [EfocalEst, IincoEst, data] = batch(u, image, darkHole, model, estimatorBatch, data);
    EfocalEst(abs(EfocalEst).^2 > 1e-2) = 0;
    data.EfocalEst0 = EfocalEst;
    data.IincoEst0 = IincoEst;
    IfocalEst = abs(EfocalEst).^2;
    contrastEst = mean(IfocalEst);
    incoherentEst = mean(IincoEst);
    data.estimatedContrastAverage0 = contrastEst;
    data.estimatedContrastMax0 = max(IfocalEst);
    data.estimatedContrastStd0 = std(IfocalEst);
    data.estimatedIncoherentAverage0 = incoherentEst;
end

disp('***********************************************************************');
disp('The initial condition');
disp(['The starting measured average contrast in the dark holes is ', num2str(mean(data.contrast0))]);
disp(['The estimated average contrast in the dark holes is ', num2str(mean(data.estimatedContrastAverage0))]);
disp('***********************************************************************');
figure(110), imagesc(log10(abs(I0))), colorbar;
caxis(cRange);
drawnow

%% Control loop start
for itr = 1 : Nitr
    if itr <= 4
        camera.exposure = 0.1;%0.01;% 0.4; %0.01
        camera.exposure0 = 0.1;%0.01;%0.4;
    elseif itr <= 10
        camera.exposure = 0.3;%0.1;%0.5; %0.1
        camera.exposure0 = 0.3;%0.1;%0.5;
    else
        camera.exposure = 0.5;%0.3;%0.9; %0.3
        camera.exposure0 =0.5;%0.3;% 0.9;
    end
    data.itr = itr;
    disp('***********************************************************************');
    disp(['Now we are running iteration ', num2str(itr) ,'/', num2str(Nitr)]);
    disp('***********************************************************************');
    
    %% compute control command
    switch controller.whichDM
        case '1'
            G = model.G1;
        case '2'
            G = model.G2;
        case 'both'
            G = cat(2, model.G1, model.G2);
        otherwise
            disp('You can only use the first DM, second DM or both for wavefront control.');
            return;
    end 
    % select the controller type
    switch lower(controller.type)
        case 'efc'
            if target.broadBandControl
                weight = ones(target.broadSampleNum, 1); % weight the importance of different wavelengths over the broadband
                M = zeros(size(G, 2), size(G, 2));
                Gx = zeros(size(G, 2), 1);
                for kWavelength = 1 : target.broadSampleNum
                    Gmon = [real(G(:, :, kWavelength)); imag(G(:, :, kWavelength))];
                    xmon = [real(EfocalEstBroadband(:, kWavelength)); imag(EfocalEstBroadband(:, kWavelength))];
                    M = M + weight(kWavelength) * (Gmon' * Gmon);
                    Gx = Gx + weight(kWavelength) * Gmon' * xmon;
                end
                command = - real((M + 1e-6 * eye(size(Gmon, 2)))^(-1)) * real(Gx);
%                 command = - real((M + controller.alpha/target.broadSampleNum * eye(size(Gmon, 2)))^(-1)) * real(Gx);
            else
                G = [real(G); imag(G)];
                x = [real(EfocalEst); imag(EfocalEst)];
                if controller.adaptiveEFC % automatically choose the regularization parameter
                    controller = adaptiveEFC(x, G, target, DM, coronagraph, camera, darkHole, controller, DM1command, DM2command, simOrLab);
                    disp(['Best alpha: ', num2str(controller.alpha)])
                end
                if controller.lineSearch && itr > 1
                    P = data.P(1:2, 1:2, :, itr-1);
                    target_contrast = max(0.1*sum(abs(x).^2)/darkHole.pixelNum, trace(mean(P, 3)));%trace(mean(P, 3));
                    alpha_set = 10.^(-7:0.1:-6);
                    result_contrast = zeros(size(alpha_set));
                    for k_alpha = 1 : length(alpha_set)
                        alpha = alpha_set(k_alpha);
                        command = EFC(x, G, alpha);
                        result_contrast(k_alpha) = sum(abs(x + G * command).^2)/darkHole.pixelNum;
                        if result_contrast(k_alpha) >= target_contrast
                            break;
                        end
                    end
                    data.control_regularization(itr) = alpha_set(k_alpha);
                    data.target_contrast_set(itr) = result_contrast(k_alpha);
                else
                    command = EFC(x, G, controller.alpha);
                end
                if itr == 1
                    contrastEst = sum(abs(x + G * command).^2)/darkHole.pixelNum + 2*estimator.stateStd0;% + estimator.processVarCoefficient * sum(command.^2);
                else
                    P = data.P(1:2, 1:2, :, itr-1);
                    contrastEst = sum(abs(x + G * command).^2)/darkHole.pixelNum + trace(mean(P, 3));% + estimator.processVarCoefficient * sum(command.^2);
                end
                
            end
        case 'robustlp'
            x = EfocalEst;
            deltaG = 3e-6;%5e-5;
            P = data.P(:, :, :, itr);
            command = robustLP(x, G, deltaG, P);
        otherwise
            disp('Currently, we only have EFC and robust Linear Programming controller. Others are still under development.')
    end
    switch controller.whichDM
        case '1'
            DM1command = DM1command + command;
        case '2'
            DM2command = DM2command + command;
        case 'both'
            DM1command = DM1command + command(1:DM.activeActNum);
            DM2command = DM2command + command(DM.activeActNum + 1 : end);
        otherwise
            disp('You can only use the first DM, second DM or both for wavefront control.');
            return;
    end
%     DM1command(DM1command > DM.voltageLimit) = DM.DM1command(DM1command > DM.voltageLimit);
%     DM2command(DM2command > DM.voltageLimit) = DM.DM2command(DM2command > DM.voltageLimit);
    data.DMcommand(:, itr) = [DM1command; DM2command];
    
    %% give the new command to DMs and take new images
%     if target.broadBandControl
%         for kWavelength = 1 : target.broadSampleNum
%             target_help.starWavelegth = target.starWavelengthBroad(kWavelength);
%             I = getImg(target_help, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
%             data.I(:, :, kWavelength, itr) = I;
%             data.measuredContrastAverage(kWavelength, itr) = mean(I(darkHole.pixelIndex));
%             data.measuredContrastMax(kWavelength, itr) = max(I(darkHole.pixelIndex));
%             data.measuredContrastStd(kWavelength, itr) = std(I(darkHole.pixelIndex));
%         end
%         disp(['The measured average contrast in the dark holes after ', num2str(itr), ' iterations is ', num2str(mean(data.measuredContrastAverage(:, itr)))]);
%     else
%         I = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
%         data.I(:,:,itr) = I;
%         data.measuredContrastAverage(itr) = mean(I(darkHole.pixelIndex));
%         data.measuredContrastMax(itr) = max(I(darkHole.pixelIndex));
%         data.measuredContrastStd(itr) = std(I(darkHole.pixelIndex));
%         disp(['The measured average contrast in the dark holes after ', num2str(itr), ' iterations is ', num2str(data.measuredContrastAverage(itr))]);
%     end
    %% for simulation, calculate the perfect contrast
    if strcmpi(simOrLab, 'simulation')
        if target.broadBandControl
            if itr == 1
                contrastPerfect = zeros(target.broadSampleNum, Nitr);
            end
            for kWavelength = 1 : target.broadSampleNum
                target_help.starWavelegth = target.starWavelengthBroad(kWavelength);
                [EfocalStarNoise, EfocalPlanetNoise, InoNoise] = opticalModel(target_help, DM, coronagraph, camera, DM1command, DM2command);
                contrastPerfect(kWavelength, itr) = mean(InoNoise(darkHole.pixelIndex));
            end
        else
            if itr == 1
                contrastPerfect = zeros(Nitr, 1);
            end
            [EfocalStarNoise, EfocalPlanetNoise, InoNoise] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
            contrastPerfect(itr) = mean(InoNoise(darkHole.pixelIndex));
            data.EfocalPerfect(:, itr) = EfocalStarNoise(darkHole.pixelIndex);
        end
    end
    %% estimate the electric field
    disp(['Running ', estimator.type, ' estimator ...']);
    if target.broadBandControl
        switch lower(estimator.type)
            case 'perfect'
                assert(strcmpi(simOrLab, 'simulation'), 'The perfect estimation can only be used in simulation!');
                EfocalEstBroadband = zeros(darkHole.pixelNum, target.broadSampleNum);
                IincoEstBroadband = zeros(darkHole.pixelNum, target.broadSampleNum);
                for kWavelength = 1 : target.broadSampleNum
                    targetmon = target;
                    targetmon.starWavelength = target.starWavelengthBroad(kWavelength);
                    [EfocalStar, EfocalPlanet, I0] = opticalModel(targetmon, DM, coronagraph, camera, DM1command, DM2command);
                    EfocalEst = EfocalStar(darkHole.pixelIndex);
                    IincoEst = abs(EfocalPlanet(darkHole.pixelIndex)).^2; % We can have perfect knowledge of the electric field in simulation
                    EfocalEstBroadband(:, kWavelength) = EfocalEst;
                    IincoEstBroadband(:, kWavelength) = IincoEst;
                end
            case 'batch'
                [imageAll, u, data] = takeProbingImagesBroad(contrastEst, target, DM, coronagraph, camera, darkHole, estimator, DM1command, DM2command, simOrLab, data);
                data.uProbe(:, :, data.itr) = u;
                EfocalEstBroadband = zeros(darkHole.pixelNum, target.broadSampleNum);
                IincoEstBroadband = zeros(darkHole.pixelNum, target.broadSampleNum);
                for kWavelength = 1 : target.broadSampleNum
                    model_help.G1 = squeeze(model.G1(:, :, kWavelength));
                    model_help.G2 = squeeze(model.G2(:, :, kWavelength));
                    image = squeeze(imageAll(:, :, :, kWavelength));
                    [EfocalEst, IincoEst, data] = batch(u, image, darkHole, model_help, estimator, data, kWavelength);
                    if itr > 5 % since the batch can be really noisy in low SNR case, zero the estimates with really high noise
                        EfocalEst(abs(EfocalEst).^2 > 1e-4) = 0;
                    end
                    EfocalEstBroadband(:, kWavelength) = EfocalEst;
                    IincoEstBroadband(:, kWavelength) = IincoEst;
                end
            case 'kalman'
                [imageAll, u, data] = takeProbingImagesBroad(contrastEst, target, DM, coronagraph, camera, darkHole, estimator, DM1command, DM2command, simOrLab, data);
                data.uProbe(:, :, data.itr) = u;
                EfocalEstBroadband = zeros(darkHole.pixelNum, target.broadSampleNum);
                IincoEstBroadband = zeros(darkHole.pixelNum, target.broadSampleNum);
                for kWavelength = 1 : target.broadSampleNum
                    model_help.G1 = squeeze(model.G1(:, :, kWavelength));
                    model_help.G2 = squeeze(model.G2(:, :, kWavelength));
                    image = squeeze(imageAll(:, :, :, kWavelength));
                    [EfocalEst, IincoEst, data] = Kalman(u, image, darkHole, model_help, estimator, controller, data, kWavelength);
                    EfocalEstBroadband(:, kWavelength) = EfocalEst;
                    IincoEstBroadband(:, kWavelength) = IincoEst;
                end
            case 'ekf'
                %%
                [imageAll, u, data] = takeProbingImagesBroad(contrastEst, target, DM, coronagraph, camera, darkHole, estimator, DM1command, DM2command, simOrLab, data);
                data.uProbe(:, :, data.itr) = u;
                EfocalEstBroadband = zeros(darkHole.pixelNum, target.broadSampleNum);
                IincoEstBroadband = zeros(darkHole.pixelNum, target.broadSampleNum);
                for kWavelength = 1 : target.broadSampleNum
                    model_help.G1 = squeeze(model.G1(:, :, kWavelength));
                    model_help.G2 = squeeze(model.G2(:, :, kWavelength));
                    image = squeeze(imageAll(:, :, :, kWavelength));
                    [EfocalEst, IincoEst, data] = EKF(u, image, darkHole, model_help, estimator, controller, data, kWavelength);
                    EfocalEstBroadband(:, kWavelength) = EfocalEst;
                    IincoEstBroadband(:, kWavelength) = IincoEst;
                end
            otherwise
                disp('Other estimators are still under development!');
                return;
        end
        data.EfocalEst(:, :, itr) = EfocalEstBroadband;
        data.IincoEst(:, :, itr) = IincoEstBroadband;
        IfocalEst = abs(EfocalEstBroadband).^2;
        contrastEst = mean(IfocalEst);
%         contrastEst = mean(IfocalEst) + mean(squeeze(data.P(1, 1, :, itr) + data.P(2, 2, :, itr)));
        incoherentEst = mean(IincoEst);
        data.estimatedContrastAverage(:, itr) = contrastEst;
        data.estimatedIncoherentAverage(:, itr) = incoherentEst;
        data.estimatedContrastMax(:, itr) = max(IfocalEst);
        data.estimatedContrastStd(:, itr) = std(IfocalEst);
    else
        switch lower(estimator.type)
            case 'perfect'
                assert(strcmpi(simOrLab, 'simulation'), 'The perfect estimation can only be used in simulation!');
                [EfocalStar, EfocalPlanet, I0] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
                EfocalEst = EfocalStar(darkHole.pixelIndex);
                IincoEst = abs(EfocalPlanet(darkHole.pixelIndex)).^2; % We can have perfect knowledge of the electric field in simulation
            case 'batch'
                [image, u, data] = takeProbingImages(contrastEst, target, DM, coronagraph, camera, darkHole, estimator, DM1command, DM2command, simOrLab, data);
                data.uProbe(:, :, data.itr) = u;
                if estimator.EKFpairProbing
                    [EfocalEst, IincoEst, data] = batch(u, image, darkHole, model, estimator, data);
                else
                    [EfocalEst, IincoEst, data] = batch2(u, image, darkHole, model, estimator, data);
                end
                if itr > 5 % since the batch can be really noisy in low SNR case, zero the estimates with really high noise
                    EfocalEst(abs(EfocalEst).^2 > 1e-4) = 0;
                else
                    EfocalEst(abs(EfocalEst).^2 > 1e-2) = 0;
                end
            case 'kalman'
                [image, u, data] = takeProbingImages(contrastEst, target, DM, coronagraph, camera, darkHole, estimator, DM1command, DM2command, simOrLab, data);
                data.uProbe(:, :, data.itr) = u;
                if estimator.EKFpairProbing
                    [EfocalEst, IincoEst, data] = Kalman(u, image, darkHole, model, estimator, controller, data);
                else
                    [EfocalEst, IincoEst, data] = Kalman2(u, image, darkHole, model, estimator, controller, data);
                end
            case 'ekf'
                [image, u, data] = takeProbingImages(contrastEst, target, DM, coronagraph, camera, darkHole, estimator, DM1command, DM2command, simOrLab, data);
                data.uProbe(:, :, data.itr) = u;
                if estimator.nonProbeImage
                    if estimator.EKFincoherent
                        [EfocalEst, IincoEst, data] = EKF(u, image, darkHole, model, estimator, controller, data);
                    else
                        [EfocalEst, IincoEst, data] = EKF4(u, image, darkHole, model, estimator, controller, data);
                    end
                else
                    if estimator.EKFincoherent
                        [EfocalEst, IincoEst, data] = EKF2(u, image, darkHole, model, estimator, controller, data);
                    else
                        [EfocalEst, IincoEst, data] = EKF3(u, image, darkHole, model, estimator, controller, data);
                    end
                end
%                 [EfocalEst, IincoEst, data] = EKF(u, image, darkHole, model, estimator, controller, data);
                if itr > 40
                    EfocalEst(abs(EfocalEst).^2 > 1e-5) = 0;
                elseif itr > 20 % since the batch can be really noisy in low SNR case, zero the estimates with really high noise
                    EfocalEst(abs(EfocalEst).^2 > 1e-4) = 0;
                else
                    EfocalEst(abs(EfocalEst).^2 > 1e-2) = 0;
                end
            otherwise
                disp('Other estimators are still under development!');
                return;
        end
%         probeImage(:, :, :, itr) = image;
        if estimator.saveData
            data.imageSet{itr} = image;
            data.probeSet{itr} = u;
        end
        data.EfocalEst(:, itr) = EfocalEst;
        data.IincoEst(:, itr) = IincoEst;
        IfocalEst = abs(EfocalEst).^2;
        contrastEst = mean(IfocalEst);
%         contrastEst = mean(IfocalEst) + mean(squeeze(data.P(1, 1, :, itr) + data.P(2, 2, :, itr)));
        incoherentEst = mean(IincoEst);
        data.estimatedContrastAverage(itr) = mean(IfocalEst);
        data.estimatedIncoherentAverage(itr) = incoherentEst;
        data.estimatedContrastMax(itr) = max(IfocalEst);
        data.estimatedContrastStd(itr) = std(IfocalEst);
    end
    disp(['The estimated average contrast in the dark holes is ', num2str(mean(contrastEst))]);
    
    %% check the contrast after giving new control commands
    if target.broadBandControl
        for kWavelength = 1 : target.broadSampleNum
            I = squeeze(imageAll(:, :, 1, kWavelength));
            data.I(:, :, kWavelength, itr) = I;
            data.measuredContrastAverage(kWavelength, itr) = mean(I(darkHole.pixelIndex));
            data.measuredContrastMax(kWavelength, itr) = max(I(darkHole.pixelIndex));
            data.measuredContrastStd(kWavelength, itr) = std(I(darkHole.pixelIndex));
        end
        disp(['The measured average contrast in the dark holes after ', num2str(itr), ' iterations is ', num2str(mean(data.measuredContrastAverage(:, itr)))]);
    else
        camera_help = camera;
        if itr >= 30
            camera_help.exposure = 10 * camera.exposure;
        end
        I = getImg(target, DM, coronagraph, camera_help, DM1command, DM2command, simOrLab);
%         I = squeeze(image(:, :, 1));
        data.I(:,:,itr) = I;
        data.measuredContrastAverage(itr) = mean(I(darkHole.pixelIndex));
        data.measuredContrastMax(itr) = max(I(darkHole.pixelIndex));
        data.measuredContrastStd(itr) = std(I(darkHole.pixelIndex));
        disp(['The measured average contrast in the dark holes after ', num2str(itr), ' iterations is ', num2str(data.measuredContrastAverage(itr))]);
    end
    %% Visualization Prep
    % focal plane estimations in log scale after giving control commands
    IincoEst2D = zeros(size(I));
    if target.broadBandControl
        IincoEst2D(darkHole.pixelIndex) = mean(data.IincoEst(:, :, itr), 2);
    else
        IincoEst2D(darkHole.pixelIndex) = IincoEst;
    end
    data.IincoEst2D(:,:,itr) = IincoEst2D;
    
    IcoEst2D = zeros(size(I));
    if target.broadBandControl
        IcoEst2D(darkHole.pixelIndex) = mean(abs(data.EfocalEst(:, :, itr)).^2, 2);
    else
        IcoEst2D(darkHole.pixelIndex) = abs(EfocalEst).^2;
    end
    data.IcoEst2D(:,:,itr) = IcoEst2D;

    if (strcmpi(simOrLab, 'simulation'))
        if target.broadBandControl
            figure(22), semilogy(0:itr, mean([data.contrast0, contrastPerfect(:, 1:itr)], 1), '-o');
        else
            figure(22), semilogy(0:itr, [data.contrast0; contrastPerfect(1:itr)], '-o');
        end
        ylim([10^(cRange(1)), 10^(cRange(2))]);
        legend('perfect');
        drawnow
    end
    
    % measured change of focal plane image
    if ~target.broadBandControl
        if itr == 1
            dImeasured = data.I(:,:,itr) - data.I0;
        else
            dImeasured = data.I(:,:,itr) - data.I(:,:,itr - 1);
        end
        dImeasured2D = zeros(size(dImeasured));
        dImeasured2D(darkHole.pixelIndex) = dImeasured(darkHole.pixelIndex);
        data.dImeasured2D(:,:,itr) = dImeasured2D;
%       
        % linear predicted change of focal plane image
        switch controller.whichDM
            case '1'
                dEmodel = model.G1 * command;
            case '2'
                dEmodel = model.G2 * command;
            case 'both'
                dEmodel = model.G1 * command(1:DM.activeActNum) + model.G2 * command(DM.activeActNum + 1 : end);
            otherwise
                disp('You can only use the first DM, second DM or both for wavefront control.');
                return;
        end
        EfocalEstNew = EfocalEst + dEmodel;
        dImodel = abs(EfocalEstNew).^2 - abs(EfocalEst).^2;
        dImodel2D = zeros(size(dImeasured));
        dImodel2D(darkHole.pixelIndex) = dImodel;
        data.dImodel2D(:,:,itr) = dImodel2D;
    end
    % Plot data
    PlotEFC
end
data.camExp = camera.exposure;
%% save data
eval([data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), '=data;']);
cd(folder.dataLibrary);
eval(['save ', data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), ' ', data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), ';']);
cd(folder.main);
% cd(folder.dataLibrary);
% eval(['save model', num2str(runTrial), ' model;']);
% cd(folder.main);

%% correct model errors - monochromatic
% initialization of the model
G1Learned = model.G1;
G2Learned = model.G2;
Qlearned = zeros(2, 2, size(model.G1, 1));
% Rlearned = zeros(2, 2, size(model.G1, 1));
Rlearned = zeros(estimator.NumImgPair, estimator.NumImgPair, size(model.G1, 1));

% clean the wavefront control data
NitrEM = 3;
uAll = data.DMcommand - [zeros(1904, 1), data.DMcommand(:, 1:end-1)];
uProbeAll = cat(1, data.uProbe, zeros(size(data.uProbe)));

%% start identifying the model of different pixels
parfor index = 1: size(model.G1, 1)
    %%
    disp(['Now we are learning pixel ', num2str(index)]);
    G = [G1Learned(index, :), G2Learned(index, :)];
    G = [real(G); imag(G)];
    % initialize the x0 and P0
    x0 = [real(data.EfocalEst0(index)); imag(data.EfocalEst0(index))];
%     P0 = 1e-5 * eye(2);
%     Q = 3e-9 * eye(2);
%     R = 3e-14 * eye(estimator.NumImgPair);
    P0 = 1e-4 * eye(2);
    Q = 3e-8 * eye(2); % might need to adjust, might need to make larger
    R = 3e-13 * eye(estimator.NumImgPair); % might need to adjust
    yAll = squeeze(data.y(index, :, :));
    % online learning
    delta1 = 1e-1; % might need to adjust, might need to make smaller
    delta2 = 1e-1; % might need to adjust, might need to make smaller
    batchSize = 3; % how many observations for each updates
    %%
    for learningItr = 1 : batchSize : 18 %max number should be largest number divisible by 3 and less than Nitr for controls
        u = uAll(:, learningItr : learningItr+batchSize-1);
        uProbe = uProbeAll(:, :, learningItr : learningItr+batchSize-1);
        y = yAll(:, learningItr : learningItr+batchSize-1);
        [system, stateEst]= onlineLearning(u, y, G, Q, R, x0, P0, uProbe, NitrEM, delta1, delta2);
        %%
        G1Learned(index, :) = system.G(1, 1:size(model.G1, 2)) + 1i * system.G(2, 1:size(model.G1, 2));
        G2Learned(index, :) = system.G(1, size(model.G1, 2)+1:end) + 1i * system.G(2, size(model.G1, 2)+1:end);
        Qlearned(:, :, index) = system.Q;
        Rlearned(:, :, index) = system.R;
        G = system.G;
        x0 = stateEst.x(:, end);
        P0 = stateEst.P(:, :, end);
    end
end
model.G1 = G1Learned;
model.G2 = G2Learned;
disp('I am done!');
% save model model
% %% correct model errors  - broadband
% modelBroadband = model;
% % initialization of the model
% G1Learned = modelBroadband.G1;
% G2Learned = modelBroadband.G2;
% Qlearned = zeros(2, 2, size(model.G1, 1), target.broadSampleNum);
% Rlearned = zeros(2, 2, size(model.G1, 1), target.broadSampleNum);
% % clean the wavefront control data
% NitrEM = 3;
% uAll = data.DMcommand - [zeros(1904, 1), data.DMcommand(:, 1:end-1)];
% uProbeAll = cat(1, data.uProbe, zeros(size(data.uProbe)));
% 
% %% start identifying the model of different pixels
% parfor index = 1: size(model.G1, 1)
%     
%     %% prepare the data
%     for kWavelength = 1 : 7
%         %%
%         disp(['Now we are learning pixel ', num2str(index), ' at wavelength ', num2str(target.starWavelengthBroad(kWavelength)), 'nm']);
% 
%         G1 = [real(modelBroadband.G1(index, :, kWavelength)); imag(modelBroadband.G1(index, :, kWavelength))];
%         G2 = [real(modelBroadband.G2(index, :, kWavelength)); imag(modelBroadband.G2(index, :, kWavelength))];
%         G = [G1, G2];
%         % initialize the x0 and P0
%         x0 = [real(data.EfocalEst0(index, kWavelength)); imag(data.EfocalEst0(index, kWavelength))];
%         P0 = 1e-5 * eye(2);
%         Q = 3e-9 * eye(2);
%         R = 3e-14 * eye(estimator.NumImgPair);
%         yAll = squeeze(data.yBroadband(index, :, kWavelength, :));
%         % online learning
%         delta1 = 1e-1;
%         delta2 = 1e-1;
%         batchSize = 3; % how many observations for each updates
%     %%
%         for learningItr = 1 : batchSize : 18
%             u = uAll(:, learningItr : learningItr+batchSize-1);
% %             uProbe = uProbeAll(:, :, learningItr+1 : learningItr+1+batchSize-1);
% %             y = yAll(:, learningItr+1 : learningItr+1+batchSize-1);
%             uProbe = uProbeAll(:, :, learningItr : learningItr+batchSize-1);
%             y = yAll(:, learningItr : learningItr+batchSize-1);
%             [system, stateEst]= onlineLearning(u, y, G, Q, R, x0, P0, uProbe, NitrEM, delta1, delta2);
%             %%
%             G1Learned(index, :, kWavelength) = system.G(1, 1:size(model.G1, 2)) + 1i * system.G(2, 1:size(model.G1, 2));
%             G2Learned(index, :, kWavelength) = system.G(1, size(model.G1, 2)+1:end) + 1i * system.G(2, size(model.G1, 2)+1:end);
%             Qlearned(:, :, index, kWavelength) = system.Q;
%             Rlearned(:, :, index, kWavelength) = system.R;
%             G = system.G;
%             x0 = stateEst.x(:, end);
%             P0 = stateEst.P(:, :, end);
%         end
% 
%     end
% end
% modelBroadband.G1 = G1Learned;
% modelBroadband.G2 = G2Learned;
% model = modelBroadband;
% end

%% introducing drift after reaching high contrast
target.drift = 1;
target.broadBandControl = 1;
cameraPerfect = camera;
cameraPerfect.noise = 0;
for kDrift = 1 : 100
    if target.broadBandControl
        target_help = target;
        Ibroadband = zeros(camera.Neta, camera.Nxi, target.broadSampleNum);
        for kWavelength = 1 : target.broadSampleNum
            target_help.starWavelegth = target.starWavelengthBroad(kWavelength);
            I = getImg(target_help, DM, coronagraph, cameraPerfect, DM1command, DM2command, simOrLab);
            Ibroadband(:, :, kWavelength) = I;
        end
        figure(101), imagesc(log10(abs(mean(Ibroadband, 3)))), colorbar;
        caxis([-10, -8]);
        drawnow
    else
        I = getImg(target, DM, coronagraph, cameraPerfect, DM1command, DM2command, simOrLab);
        figure(101), imagesc(log10(abs(I))), colorbar;
        caxis([-10, -8]);
        drawnow
    end
    target = targetDrift(target);
end