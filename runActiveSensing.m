
clc;
% clear;
close all;

%% Initialize the system and parameters
% for runTrial = 1 : 50
Nitr = 40;%4000; % iterations of control loop
cRange = [-8, -4]; %[-12, -3];% the range for display
simOrLab = 'simulation'; % 'simulation' or 'lab', run the wavefront correction loops in simulation or in lab
% runTrial = 4;
Initialization;

%% Initialize the hardware driver if we are running experiment
% Laser_Enable('on');
% Laser_Power(65, 1);
if (strcmpi(simOrLab, 'lab')) % if conducting experiment in lab, initialize DM and camera drivers
%     DM.DM1bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_CLOSED_LOOP_200nm_VOLTAGES.txt','-ascii');
    DM.DM1bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW004#14_CLOSED_LOOP_200nm_Voltages_DM#1.txt','-ascii');
    DM.DM1bias = DM.DM1bias(1 : DM.activeActNum); % flatten map voltages in volts on DM1
%     DM.DM2bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#014_CLOSED_LOOP_200nm_Voltages.txt','-ascii');
    DM.DM2bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW018#40_CLOSED_LOOP_200nm_Voltages_DM#2.txt','-ascii');
    DM.DM2bias = DM.DM2bias(1 : DM.activeActNum); % flatten map voltages in volts on DM2
    initializeDM(DM);
    camera = initializeCamera(camera);
end
%% disconnect the camera at the end
disconnecting = 0;
if disconnecting == 1
	error = BurstHVA4096Frame1D(1, zeros(4096,1)); % finalize DMs
	finalizeCamera(camera) % finalize the camera
%     Laser_Power(0, 1)
%     Laser_Enable('off')    
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
%     model = stateSpace(target, DM, coronagraph, camera, darkHole);
    load model.mat
end

%% define the active sensor
G1real = py.numpy.array(real(model.G1));
G1imag = py.numpy.array(imag(model.G1));
G2real = py.numpy.array(real(model.G2));
G2imag = py.numpy.array(imag(model.G2));
switch lower(estimator.type)
    case 'kalman'
        if strcmpi(estimator.whichDM, 'both')
            estimator.sensor = py.sensing.active_sensing2(G1real, G1imag, G2real, G2imag, estimator.NumImgPair, 2, ...
                estimator.Q1, estimator.Q3, estimator.R1, estimator.R3);
        else
            estimator.sensor = py.sensing.active_sensing2(G1real, G1imag, G2real, G2imag, estimator.NumImgPair, 1, ...
                    estimator.Q1, estimator.Q3, estimator.R1, estimator.R3);
        end
    case 'ekf'
        if strcmpi(estimator.whichDM, 'both')
            estimator.sensor = py.sensing.active_sensing4(G1real, G1imag, G2real, G2imag, estimator.NumImg, 2, ...
                estimator.Q1, estimator.Q3, estimator.Q4, estimator.Q5, estimator.R1, estimator.R3);
        else
            estimator.sensor = py.sensing.active_sensing4(G1real, G1imag, G2real, G2imag, estimator.NumImg, 1, ...
                estimator.Q1, estimator.Q3, estimator.Q4, estimator.Q5, estimator.R1, estimator.R3);
        end
    otherwise
        disp('Not using active sensing!')
end
%%
% for kCorrection = 2 : 10
%     runTrial = kCorrection;
%% take focal plane image with no DM poking
camera.exposure = 0.01;
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
%     data.IincoEst0 = IincoEst;
    data.IincoEst0 = zeros(size(IincoEst));
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
figure(1), imagesc(log10(abs(I0))), colorbar;
caxis(cRange);
drawnow

%% Control loop start
for itr = 1 : Nitr
    if itr <= 0
        camera.exposure = 0.03;
    else
        camera.exposure = 0.1;
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
                end
                command = EFC(x, G, controller.alpha);
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
                EfocalPerfect = zeros(darkHole.pixelNum, Nitr);
            end
            if itr > 1
                EfocalStarNoiseOld = EfocalStarNoise;
            end
            [EfocalStarNoise, EfocalPlanetNoise, InoNoise] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
            contrastPerfect(itr) = mean(InoNoise(darkHole.pixelIndex));
            EfocalPerfect(:, itr) = EfocalStarNoise(darkHole.pixelIndex);
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
                [EfocalEst, IincoEst, data] = batch(u, image, darkHole, model, estimator, data);
                if itr > 5 % since the batch can be really noisy in low SNR case, zero the estimates with really high noise
                    EfocalEst(abs(EfocalEst).^2 > 1e-4) = 0;
                else
                    EfocalEst(abs(EfocalEst).^2 > 1e-2) = 0;
                end
            case 'kalman'
               if itr == 1
                    E_past = data.EfocalEst0;
                    P_past = zeros(2, 2, darkHole.pixelNum);
                    P_past(1, 1, :) = estimator.stateStd0;
                    P_past(2, 2, :) = estimator.stateStd0;
                else
                    E_past = data.EfocalEst(:, itr-1);
                    P_past = zeros(2, 2, darkHole.pixelNum);
                    P_past(1:2, 1:2, :) = data.P(1:2, 1:2, :, itr-1);
                end
                
                contrast_past = mean(abs(E_past).^2 + squeeze(P_past(1, 1, :) + P_past(2, 2, :)));
                E_pred = E_past + model.G1 * command(1:952) + model.G2 * command(953:end);
                contrast_pred = mean(abs(E_pred).^2 + squeeze(P_past(1, 1, :) + P_past(2, 2, :))) + 2*(sum(command.^2) + 0.3) * estimator.processVarCoefficient;
                disp(['Past contrast: ', num2str(contrast_past), ' Predicted contrast: ', num2str(contrast_pred)]);
                
                E_est_past_real = py.numpy.array(real(E_past));
                E_est_past_imag = py.numpy.array(imag(E_past));
                P_past_permuted = permute(P_past, [3, 1, 2]);
                P_est_past = py.numpy.array(P_past_permuted);
                command1 = py.numpy.array(command(1:952));
                command2 = py.numpy.array(command(953:end));
                if estimator.activeSensing

                    beta = estimator.beta;
                    rate = estimator.rate;
                    sgd_itr = estimator.sgd_itr;

                    temp = py.sensing.optimal_probe2(estimator.sensor, E_est_past_real, E_est_past_imag, P_est_past, command1, command2, beta, rate, sgd_itr);
                    if strcmpi(estimator.whichDM, 'both')
                        temp2 = double(py.numpy.squeeze(temp{1}, int8(0)));
                        temp3 = double(py.numpy.squeeze(temp{2}, int8(0)));
                        u = [temp2, temp3]';
                    else
                        temp2 = double(py.numpy.squeeze(temp{1}, int8(0)));
                        u = temp2';
                    end
                    
                    
                    image = zeros(camera.Neta, camera.Nxi, 1 + 2*estimator.NumImgPair); % unprobed and probed images
                    image(:, :, 1) = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
                    for kProbe = 1 : estimator.NumImgPair
                        if strcmpi(estimator.whichDM, 'both')
                            image(:, :, 2*kProbe) = getImg(target, DM, coronagraph, camera, DM1command + u(1:DM.activeActNum, kProbe), DM2command + u(DM.activeActNum+1:end, kProbe), simOrLab);
                            image(:, :, 2*kProbe+1) = getImg(target, DM, coronagraph, camera, DM1command - u(1:DM.activeActNum, kProbe), DM2command - u(DM.activeActNum+1:end, kProbe), simOrLab);
                        else
                            image(:, :, 2*kProbe) = getImg(target, DM, coronagraph, camera, DM1command + u(:, kProbe), DM2command, simOrLab);
                            image(:, :, 2*kProbe+1) = getImg(target, DM, coronagraph, camera, DM1command - u(:, kProbe), DM2command, simOrLab);
                        end
                    end
                else
                    [image, u, data] = takeProbingImages(contrastEst, target, DM, coronagraph, camera, darkHole, estimator, DM1command, DM2command, simOrLab, data);
%                     up_values = py.numpy.array(u);
%                     beta = estimator.beta;
%                     cost = py.sensing.probe_cost(estimator.sensor, E_est_past_real, E_est_past_imag, P_est_past, command1, command2, beta, up_values);
%                     disp(['cost: ', num2str(cost)])
                end
                data.uProbe(:, :, data.itr) = u;
                [EfocalEst, IincoEst, data] = Kalman2(u, image, darkHole, model, estimator, controller, data);
                figure(5), plot(u);
                
%                 [image, u, data] = takeProbingImages(contrastEst, target, DM, coronagraph, camera, darkHole, estimator, DM1command, DM2command, simOrLab, data);
%                 data.uProbe(:, :, data.itr) = u;
%                 [EfocalEst, IincoEst, data] = Kalman(u, image, darkHole, model, estimator, controller, data);
            case 'ekf'
                if itr == 1
                    E_past = data.EfocalEst0;
                    P_past = zeros(3, 3, darkHole.pixelNum);
                    P_past(1, 1, :) = estimator.stateStd0;
                    P_past(2, 2, :) = estimator.stateStd0;
                    P_past(3, 3, :) = estimator.incoherentStd0;
                else
                    E_past = data.EfocalEst(:, itr-1);%EfocalStarNoiseOld(darkHole.pixelIndex);%
%                     P_past = zeros(3, 3, darkHole.pixelNum);
%                     P_past(1:2, 1:2, :) = data.P(1:2, 1:2, :, itr-1);
%                     P_past(3, 3, :) = 1e-14;
                    P_past = data.P(:, :, :, itr-1);
                end
                contrast_past = mean(abs(E_past).^2 + squeeze(P_past(1, 1, :) + P_past(2, 2, :)));
                E_pred = E_past + model.G1 * command(1:952) + model.G2 * command(953:end);
                Q_pred = zeros(3, 3, darkHole.pixelNum);
                Q_pred(1, 1, :) = (sum(command.^2) + 0.3) * estimator.processVarCoefficient;
                Q_pred(2, 2, :) = (sum(command.^2) + 0.3) * estimator.processVarCoefficient;
                if itr == 1
                    Q_pred(3, 3, :) = max(estimator.incoherentStd^2, 0.1 * mean(data.IincoEst0.^2));
                else
                    Q_pred(3, 3, :) = max(estimator.incoherentStd^2, 0.1 * mean(data.IincoEst(:, data.itr-1).^2));
                end
                P_pred = P_past + Q_pred;
                
                contrast_pred = mean(abs(E_pred).^2 + squeeze(P_pred(1, 1, :) + P_pred(2, 2, :)));
                disp(['Past contrast: ', num2str(contrast_past), ' Predicted contrast: ', num2str(contrast_pred)])
                
                E_est_past_real = py.numpy.array(real(E_past));
                E_est_past_imag = py.numpy.array(imag(E_past));
                P_past_permuted = permute(P_past, [3, 1, 2]);
                P_est_past = py.numpy.array(P_past_permuted);
                E_pred_real = py.numpy.array(real(E_pred));
                E_pred_imag = py.numpy.array(imag(E_pred));
                P_pred_permuted = permute(P_pred, [3, 1, 2]);
                P_pred_est = py.numpy.array(P_pred_permuted);
                
                command1 = py.numpy.array(command(1:952));
                command2 = py.numpy.array(command(953:end));
                
                if estimator.activeSensing

                    beta = estimator.beta;
                    rate = estimator.rate;
                    sgd_itr = estimator.sgd_itr;
%                     temp = py.sensing.optimal_probe(estimator.sensor, E_est_past_real, E_est_past_imag, P_est_past, command1, command2, beta, rate, sgd_itr);
%                     temp = py.sensing.optimal_probe3(estimator.sensor, E_est_past_real, E_est_past_imag, P_est_past, command1, command2, beta, rate, sgd_itr);
                    temp = py.sensing.optimal_probe4(estimator.sensor, E_pred_real, E_pred_imag, P_pred_est, beta, rate, sgd_itr);
                    
                    if strcmpi(estimator.whichDM, 'both')
                        temp2 = double(py.numpy.squeeze(temp{1}, int8(0)));
                        temp3 = double(py.numpy.squeeze(temp{2}, int8(0)));
                        u = [temp2, temp3]';
                    else
                        temp2 = double(py.numpy.squeeze(temp{1}, int8(0)));
                        u = temp2';
                    end
%                   

                    
%                     sensor = estimator.sensor;
%                     optimal_offset;
%                     u = opt_up;
        
                    image = zeros(camera.Neta, camera.Nxi, 1 + estimator.NumImg); % unprobed and probed images
                    image(:, :, 1) = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
                    for kProbe = 1 : estimator.NumImg
                        if strcmpi(estimator.whichDM, 'both')
                            image(:, :, kProbe+1) = getImg(target, DM, coronagraph, camera, DM1command + u(1:DM.activeActNum, kProbe), DM2command + u(DM.activeActNum+1:end, kProbe), simOrLab);
                        else
                            image(:, :, kProbe+1) = getImg(target, DM, coronagraph, camera, DM1command + u(:, kProbe), DM2command, simOrLab);
                        end
                    end
                else
                    [image, u, data] = takeProbingImages(contrastEst, target, DM, coronagraph, camera, darkHole, estimator, DM1command, DM2command, simOrLab, data);
                    up_values = py.numpy.array(u);
                    beta = estimator.beta;
                    cost = py.sensing.probe_cost4(estimator.sensor, E_pred_real, E_pred_imag, P_pred_est, beta, up_values);
                    disp(['cost: ', num2str(cost)])
                end
                data.uProbe(:, :, data.itr) = u;
                if estimator.nonProbeImage
                    [EfocalEst, IincoEst, data] = EKF(u, image, darkHole, model, estimator, controller, data);
                else
                    [EfocalEst, IincoEst, data] = EKF2(u, image, darkHole, model, estimator, controller, data);
                end
                figure(5), plot(u);
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
        IfocalEst = abs(EfocalEst).^2;% + squeeze(data.P(1, 1, :, itr) + data.P(2, 2, :, itr));
        IfocalEst_err = 0;
        for pixelInd = 1 : darkHole.pixelNum
            temp = [real(EfocalEst(pixelInd)); imag(EfocalEst(pixelInd))];
            IfocalEst_err = IfocalEst_err + 2 * (temp'*data.P(1:2, 1:2, pixelInd, itr)*temp + trace(data.P(1:2, 1:2, pixelInd, itr)^2));
        end
        IfocalEst_err = sqrt(IfocalEst_err);
        IfocalEst_err = IfocalEst_err / darkHole.pixelNum;
        contrastEst = mean(IfocalEst);
        incoherentEst = mean(IincoEst);
        data.estimatedContrastAverage(itr) = contrastEst;
        data.estimatedIncoherentAverage(itr) = incoherentEst;
        data.estimatedContrastMax(itr) = max(IfocalEst);
        data.estimatedContrastStd(itr) = std(IfocalEst);
        data.estimatedContrastErr(itr) = IfocalEst_err;
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
        if itr >= 10
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
    %% Visualizations
    % focal plane estimations in log scale after giving control commands
    IincoEst2D = zeros(size(I));
    if target.broadBandControl
        IincoEst2D(darkHole.pixelIndex) = mean(data.IincoEst(:, :, itr), 2);
    else
        IincoEst2D(darkHole.pixelIndex) = IincoEst;
    end
    figure(10), imagesc(log10(abs(IincoEst2D))), colorbar;
    caxis(cRange);
    title(['Incoherent light after control iteration ', num2str(itr)]);
    drawnow
    
    IcoEst2D = zeros(size(I));
    if target.broadBandControl
        IcoEst2D(darkHole.pixelIndex) = mean(abs(data.EfocalEst(:, :, itr)).^2, 2);
    else
        IcoEst2D(darkHole.pixelIndex) = abs(EfocalEst).^2;
    end
    figure(11), imagesc(log10(abs(IcoEst2D))), colorbar;
    caxis(cRange);
    title(['Coherent light after control iteration ', num2str(itr)]);
    drawnow
    
    % focal plane images given control commands in log scale
    if target.broadBandControl
        figure(1), imagesc(log10(abs(mean(data.I(:, :, :, itr), 3)))), colorbar
    else
        figure(1), imagesc(log10(abs(I))), colorbar;
    end
    caxis(cRange);
    title(['After control iteration ', num2str(itr)]);
    drawnow
    
    % contrast correction curve - average
    if target.broadBandControl
        figure(2), semilogy(0:itr, mean([data.contrast0, data.measuredContrastAverage(:, 1:itr)], 1), '-o' ,0:itr, mean([data.estimatedContrastAverage0, data.estimatedContrastAverage(:, 1:itr)], 1), '-s', 0:itr, mean([data.estimatedIncoherentAverage0, data.estimatedIncoherentAverage(:, 1:itr)], 1), '-^');
    else
        figure(2), semilogy(0:itr, [data.contrast0; data.measuredContrastAverage(1:itr)], '-o' ,0:itr, [data.estimatedContrastAverage0; data.estimatedContrastAverage(1:itr)], '-s', 0:itr, [data.estimatedIncoherentAverage0; data.estimatedIncoherentAverage(1:itr)], '-^');
    end
    ylim([10^(cRange(1)), 10^(cRange(2))]);
    legend('measured', 'estimated', 'incoherent');
    drawnow
    if (strcmpi(simOrLab, 'simulation'))
        if target.broadBandControl
            figure(22), semilogy(0:itr, mean([data.contrast0, contrastPerfect(:, 1:itr)], 1), '-o');
        else
            figure(22), semilogy(0:itr, [data.contrast0; contrastPerfect(1:itr)], '-o');
        end
        ylim([10^(cRange(1)), 10^(cRange(2))]);
        xlim([0, itr]);
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
        figure(3), imagesc(log10(abs(dImeasured2D))), colorbar;
        title('Measured Intensity Change');
        caxis(cRange);
        drawnow

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
        figure(4), imagesc(log10(abs(dImodel2D))), colorbar;
        title('Model-predicted Intensity Change');
        caxis(cRange);
        drawnow
    end
end

%% save data
eval([data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'SimTrial', num2str(runTrial), '=data;']);
cd(folder.dataLibrary);
eval(['save ', data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'SimTrial', num2str(runTrial), ' ', data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'SimTrial', num2str(runTrial), ';']);
cd(folder.main);
% eval(['save model', num2str(kCorrection), ' model;']);
% cd(folder.main);
% end