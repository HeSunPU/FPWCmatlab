%% Sec #0 - main function of broadband wavefront control
% revised from runEFC.m by He Sun on Jul. 30, 2019

clc;
clear;
close all;

%% Sec #1 - Initialize the system and parameters
Nitr = 5;%4000; % iterations of control loop
cRange = [-7, -3]; %[-12, -3];% the range for display
simOrLab = 'simulation';%'simulation';%  'simulation' or 'lab', run the wavefront correction loops in simulation or in lab
runTrial = 1;
systemID_flag = true;%false;%
n_systemID = 3;%10;
if ~systemID_flag
    n_systemID = 1;
end
%clear folder
Initialization_broadband;

%% Sec #2 - Initialize the hardware driver if we are running experiment
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
    if strcmpi(camera.name, 'QSI')
        camera = initializeCamera(camera);
    elseif strcmpi(camera.name, 'Starlight')
        startMaxim;
        NpxX = 1392;
        NpxY = 1040;
        Xc = NpxX/2;Yc = NpxY/2; 
%         RX = NpxX/2;RY = NpxY/2; 
%         RX = NpxX/4;RY = NpxY/4; 
%         RX = 512;RY = 512;
        RX = 568;RY = 512;
        spaxelX = 38;%29.7;% % TBC
        camPitch = camera.pitch;
        spaxelperlamD = 2;% *.9;%
        lamDps = 1/spaxelperlamD;
        lamDpp = 2*lamDps/spaxelX; % Nyquist sampled
        pixperlamD = 1/lamDpp;
        mask.rangeR = [5, 11];
        mask.rangeAngle = 40;%42.5;
        exp = 1000 * camera.exposure;%
        Nimg = camera.stacking;%
        camera.handle = hCam;
        camera.Xc = Xc;
        camera.Yc = Yc;
        camera.RX = RX;
        camera.RY = RY;
        
    end
%     fopen(target.laser)
end
%% Sec #3 - disconnect the camera at the end
disconnecting =2;
if disconnecting == 1
	error = BurstHVA4096Frame1D(1, zeros(4096,1)); % finalize DMs
	finalizeCamera(camera) % finalize the camera
    Laser_Power(0, 1)
    Laser_Enable('off')
elseif disconnecting == 2
    error = BurstHVA4096Frame1D(1, zeros(4096,1)); % finalize DMs
end
%% Sec #4 - Compute the state space model of the system
% parpool(16);
G1Broadband = zeros(darkHole.pixelNum, DM.activeActNum, target.broadSampleNum);
G2Broadband = zeros(darkHole.pixelNum, DM.activeActNum, target.broadSampleNum);
target_help = target;
for kWavelength = 1 : target.broadSampleNum
    target_help.starWavelength = target.starWavelengthBroad(kWavelength);
    target_help.normalization = target.normalizationBroadband(kWavelength);
    model = stateSpace(target_help, DM, coronagraph, camera, darkHole);
    G1Broadband(:, :, kWavelength) = model.G1;
    G2Broadband(:, :, kWavelength) = model.G2;
end
model.G1 = G1Broadband;
model.G2 = G2Broadband;
% load modelBroadband.mat
% model = modelBroadband;
% % indices = [1, 2, 4, 5, 6, 7, 8];
% indices = [1];
% 
% model.G1 = model.G1(:, :, indices);
% model.G2 = model.G2(:, :, indices);
% 
%% Sec #5 - start the system ID loop
for k_EM = 1 : n_systemID
    
%% Sec #5.1 - take focal plane image with no DM poking
DM1command = zeros(DM.activeActNum, 1);
DM2command = zeros(DM.activeActNum, 1);
% DM1command = DM1command_final;
% DM2command = DM2command_final;
if strcmpi(camera.name, 'QSI')
    camera.exposure = 5;%0.01;
    camera.exposure0 = 5;%0.01;
    target_help = target;
    for kWavelength = 1 : target.broadSampleNum
        target_help.starWavelength = target.starWavelengthBroad(kWavelength);
        target_help.normalization = target.normalizationBroadband(kWavelength);
        target_help.flux = target.fluxBroadband(kWavelength);
        if strcmpi(simOrLab, 'lab')
            fprintf(target.laser,['pos=', num2str(target.channel(kWavelength))]);
            pause(2)
        end
        if strcmpi(camera.name, 'QSI')
            I0 = getImg(target_help, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
            contrast0 = mean(I0(darkHole.pixelIndex));
            contrast0Max = max(I0(darkHole.pixelIndex));
            contrast0Std = std(I0(darkHole.pixelIndex));
            data.contrast0(kWavelength) = contrast0;
            data.contrast0Max(kWavelength) = contrast0Max;
            data.contrast0Std(kWavelength) = contrast0Std;
            data.I0(:, :, kWavelength) = I0;
        end
    end
elseif strcmpi(camera.name, 'Starlight')
    camera.exposure = 5;%0.01;
    camera.exposure0 = 5;%0.01;
%     [imgIFS, datacube] = takeIFSImg(camera);
%     cubeRotate = rotateAndCropIFS(datacube, camera);
    [imgIFS, cube] = takeIFSImgNorm(target, DM, camera, DM1command, DM2command);
    data.IFSimage0 = imgIFS;
    for kWavelength = 1 : target.broadSampleNum
        I0 = cube(:, :, kWavelength);
        contrast0 = mean(I0(darkHole.pixelIndex));
        contrast0Max = max(I0(darkHole.pixelIndex));
        contrast0Std = std(I0(darkHole.pixelIndex));
        data.contrast0(kWavelength) = contrast0;
        data.contrast0Max(kWavelength) = contrast0Max;
        data.contrast0Std(kWavelength) = contrast0Std;
        data.I0(:, :, kWavelength) = I0;
    end
else
    disp('Camera name is neither QSI or Starlight!!')
end
%% estimate the starting contrast using batch process estimation
contrastEst = -1;
data.itr = 0;
[imageAll, u, data] = takeProbingImagesBroad(contrastEst, target, DM, coronagraph, camera, darkHole, estimatorBatch, DM1command, DM2command, simOrLab, data);
data.Ip0 = imageAll;
data.uProbe0 = u;

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

disp('***********************************************************************');
disp('The initial condition');
disp(['The starting measured average contrast in the dark holes is ', num2str(mean(data.contrast0))]);
disp(['The estimated average contrast in the dark holes is ', num2str(mean(data.estimatedContrastAverage0))]);
disp('***********************************************************************');
figure(1), imagesc(log10(abs(I0))), colorbar;
caxis(cRange);
drawnow

%% Sec #5.2 - Control loop start
for itr = 1 : Nitr
    if itr <= 0
        camera.exposure = 1;%0.01;
        camera.exposure0 = 1;%0.01;
%         camera.darkFrame = 516;%dark1;
    else
        camera.exposure = 5;%0.1;
        camera.exposure0 = 5;%0.1;
%         camera.darkFrame = dark10;
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
    weight = ones(target.broadSampleNum, 1); % weight the importance of different wavelengths over the broadband
    M = zeros(size(G, 2), size(G, 2));
    Gx = zeros(size(G, 2), 1);
    for kWavelength = 1 : target.broadSampleNum
        Gmon = [real(G(:, :, kWavelength)); imag(G(:, :, kWavelength))];
        xmon = [real(EfocalEstBroadband(:, kWavelength)); imag(EfocalEstBroadband(:, kWavelength))];
        M = M + weight(kWavelength) * (Gmon' * Gmon);
        Gx = Gx + weight(kWavelength) * Gmon' * xmon;
    end
    command = - real((M + controller.alpha * eye(size(Gmon, 2)))^(-1)) * real(Gx);
    %% 
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

    data.DMcommand(:, itr) = [DM1command; DM2command];

    %% for simulation, calculate the perfect contrast
    if strcmpi(simOrLab, 'simulation')
        if itr == 1
            contrastPerfect = zeros(target.broadSampleNum, Nitr);
        end
        for kWavelength = 1 : target.broadSampleNum
            target_help.starWavelength = target.starWavelengthBroad(kWavelength);
            target_help.normalization = target.normalizationBroadband(kWavelength);
            target_help.flux = target.fluxBroadband(kWavelength);
            [EfocalStarNoise, EfocalPlanetNoise, InoNoise] = opticalModel(target_help, DM, coronagraph, camera, DM1command, DM2command);
            contrastPerfect(kWavelength, itr) = mean(InoNoise(darkHole.pixelIndex));
        end
    end
    %% estimate the electric field
    disp(['Running ', estimator.type, ' estimator ...']);
    switch lower(estimator.type)
        case 'perfect'
            assert(strcmpi(simOrLab, 'simulation'), 'The perfect estimation can only be used in simulation!');
            EfocalEstBroadband = zeros(darkHole.pixelNum, target.broadSampleNum);
            IincoEstBroadband = zeros(darkHole.pixelNum, target.broadSampleNum);
            for kWavelength = 1 : target.broadSampleNum
                target_help.starWavelength = target.starWavelengthBroad(kWavelength);
                target_help.normalization = target.normalizationBroadband(kWavelength);
                target_help.flux = target.fluxBroadband(kWavelength);
                [EfocalStar, EfocalPlanet, I0] = opticalModel(target_help, DM, coronagraph, camera, DM1command, DM2command);
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
                if itr > 20 % since the batch can be really noisy in low SNR case, zero the estimates with really high noise
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
    data.Ip(:, :, :, :, itr) = imageAll;
    disp(['The estimated average contrast in the dark holes is ', num2str(mean(contrastEst))]);
    
    %% check the contrast after giving new control commands
    for kWavelength = 1 : target.broadSampleNum
        I = squeeze(imageAll(:, :, 1, kWavelength));
        data.I(:, :, kWavelength, itr) = I;
        data.measuredContrastAverage(kWavelength, itr) = mean(I(darkHole.pixelIndex));
        data.measuredContrastMax(kWavelength, itr) = max(I(darkHole.pixelIndex));
        data.measuredContrastStd(kWavelength, itr) = std(I(darkHole.pixelIndex));
    end
    disp(['The measured average contrast in the dark holes after ', num2str(itr), ' iterations is ', num2str(mean(data.measuredContrastAverage(:, itr)))]);

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

%% Sec #5.3 - save data and update the model
if systemID_flag
    eval([data.controllerType, coronagraph.type, camera.name, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), 'EM', num2str(k_EM), '=data;']);
    cd(folder.dataLibrary);
    eval(['save ', data.controllerType, coronagraph.type, camera.name, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), 'EM', num2str(k_EM), ' ', data.controllerType, coronagraph.type, camera.name, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), 'EM', num2str(k_EM), ';']);
    cd(folder.main);
    figure(1001), semilogy(0:Nitr, mean([data.contrast0, data.measuredContrastAverage(:, 1:itr)], 1));
    hold on
    legend
    %% correct model errors using FPWCpy:HCIL_broadband_EM.py
    dataIFS.I = zeros(darkHole.pixelNum, target.broadSampleNum, 2*estimator.NumImgPair+1, Nitr+1);
    for j = 1 : 2*estimator.NumImgPair+1
        for n = 1 : target.broadSampleNum
            temp = data.Ip0(:, :, n, j);
            dataIFS.I(:, n, j, 1) = temp(darkHole.pixelIndex);
        end
    end
    for k = 1 : Nitr
        for j = 1 : 2*estimator.NumImgPair+1
            for n = 1 : target.broadSampleNum
                temp = data.Ip(:, :, n, j, k);
                dataIFS.I(:, n, j, k+1) = temp(darkHole.pixelIndex);
            end
        end
    end
    dataIFS.DMcommand = data.DMcommand;
    dataIFS.uProbe = zeros(DM.activeActNum, estimator.NumImgPair, Nitr+1);
    dataIFS.uProbe(:, :, 1) = data.uProbe0;
    dataIFS.uProbe(:, :, 2:Nitr+1) = data.uProbe;
    eval(['dataIFS', num2str(k_EM), '=dataIFS']);
    cd(folder.dataLibrary);
    eval(['save ', 'dataIFS', num2str(k_EM), ' ', 'dataIFS', num2str(k_EM)]);
    cd(folder.main);
    %% load the newly identified Jacobian model
    while ~exist([folder.dataLibrary, '/G_broadband', num2str(k_EM), '.mat'])
        pause(1);
    end
    pause(2);
    eval(['load ', '''', folder.dataLibrary, '/G_broadband', num2str(k_EM), '.mat', '''']);
    model.G1 = G_broadband(:, 1:DM.activeActNum, :);
    model.G2 = G_broadband(:, DM.activeActNum+1:end, :);
    
else
    eval([data.controllerType, coronagraph.type, camera.name, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), '=data;']);
    cd(folder.dataLibrary);
    eval(['save ', data.controllerType, coronagraph.type, camera.name, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), ' ', data.controllerType, coronagraph.type, camera.name, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), ';']);
    cd(folder.main);
end

end