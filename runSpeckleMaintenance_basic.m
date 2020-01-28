%% Updates
% Oct 9, 2019: updated to estimate OL electric field, doesnt work yet

%% Set number of iterations

Nitr = 50;
cRange = [-8, -4]; %[-12, -3];% the range for display
simOrLab ='lab';

%% Load dark hole DM commands

data_DH = data;
clear data
% save drift DM command as a different variable
DM1command_DH = data_DH.DMcommand(1:DM.activeActNum ,end);
DM2command_DH = data_DH.DMcommand(DM.activeActNum + 1 : end,end);



%% run initialization again
Initialization_Maint

Initialization_Alt
%  set stuff up for maintainin DH
DM1command = data_DH.DMcommand(1:DM.activeActNum,end);
DM2command = data_DH.DMcommand(DM.activeActNum + 1 : end,end);

data.DHcommand = [DM1command;DM2command];

estimator.type = 'ekf_speckle';
estimator.nonProbeImage = 0; % no probe images
% estimator.EKFincoherent = 1; % account for incoherent light
estimator.NumImg = 1;


data.I0 = data_DH.I(:,:,end);
data.contrast0 = data_DH.measuredContrastAverage(end); %check this one
data.measuredContrastAverageLiveOL(itrOL,1) = data.contrast0;
data.contrast0Max = data_DH.measuredContrastMax(end);
data.contrast0Std = data_DH.measuredContrastStd(end);
% estimate the starting contrast using batch process estimation
data.EfocalEst0 = data_DH.EfocalEst(:,end);
% data.EfocalEst0 = data_DH.EfocalPerfect(:, end) ; %THIS MIGHT BE WRONG


% data.IincoEst0 = data_DH.IincoEst(:,end);
data.IincoEst0 = zeros(size(data_DH.IincoEst(:,1))); %****

data.estimatedContrastAverage0 = data_DH.estimatedContrastAverage(end);
data.estimatedContrastMax0 = data_DH.estimatedContrastMax(end);
data.estimatedContrastStd0 = data_DH.estimatedContrastStd(end);
data.estimatedIncoherentAverage0 = data_DH.estimatedIncoherentAverage(end);


EfocalEst = data.EfocalEst0;
IincoEst = data.IincoEst0;
IfocalEst = abs(EfocalEst).^2;
contrastEst = mean(IfocalEst);
incoherentEst = mean(data.IincoEst0);

% Set up initial things for checking estimate

dataAlt.EfocalEst0 = data_DH.EfocalEst(:,end); % CORRECT ONE
% dataAlt.EfocalEst0 = data_DH.EfocalPerfect(:, end) ; %USE FOR DEBUGGING

%% Control loop start
for itr = 1 : Nitr %should start this a 2 and set data.DMcommand(:,:,1) to initial thing?
    if itr <= 3
        camera.exposure = 0.5;%0.01;
        camera.exposure0 = 0.5;%0.01;
    elseif itr <= 15
        camera.exposure = 0.5;
        camera.exposure0 = 0.5;
    else
        camera.exposure = 0.5;
        camera.exposure0 = 0.5;
    end
    
    data.itr = itr;
    data.image_exposure(itr,1) = camera.exposure;
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
            else
                G = [real(G); imag(G)]; %%
                x = [real(EfocalEst); imag(EfocalEst)];
                if controller.adaptiveEFC % automatically choose the regularization parameter
                end
                if controller.lineSearch && itr > 1 %constraint that enforces the target contrast larger than estimation covariance
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
                    command = EFC(x, G, controller.alpha); %%
                end
                if itr == 1
                    contrastEst = sum(abs(x + G * command).^2)/darkHole.pixelNum + 2*estimator.stateStd0;% + estimator.processVarCoefficient * sum(command.^2);
                else
                    P = data.P(1:2, 1:2, :, itr-1);
                    contrastEst = sum(abs(x + G * command).^2)/darkHole.pixelNum + trace(mean(P, 3));% + estimator.processVarCoefficient * sum(command.^2);
                end
            end
        otherwise
            disp('Currently, we only have EFC and robust Linear Programming controller. Others are still under development.')
    end
    
    data.efcCommand(:,itr) = command;
    
    %% Susan edited this block a bit
    % find mag of EFC command or contrast and scale appropriately (dither
    % should be smaller but not a lot smaller?)
    switch controller.whichDM
        case '1'
            command_mean = mean(abs(command - DM1command));
        case '2'
            command_mean = mean(abs(command - DM2command));
    end
    estimator.ditherStd = estimator.ditherScaling*10^(floor(log10(command_mean))-1); %THIS LINE ADDED
    
    %     estimator.ditherStd = 250*10^(floor(log10(command_mean))-1); %
    %     estimator.ditherStd = 500*10^(floor(log10(command_mean))-1); % HIGH
    %     ENOUGH SUCH THAT EKF DOESNT DIVERGE
    
    %     estimator.ditherStd = 50*10^(floor(log10(command_mean))-1); % OP2, PAIR WITH DRIFT OP2
    %     estimator.ditherStd = 10*10^(floor(log10(command_mean))-1); % OP1, PAIR WITH DRIFT OP1
    
    command_dither = [normrnd(0,estimator.ditherStd,[DM.activeActNum,1]);...
        normrnd(0,estimator.ditherStd,[DM.activeActNum,1])];%THIS IS DITHER COMMAND, check that it is within the resolution of the system
    %
    %     command_dither = zeros(2*DM.activeActNum,1);
    
    switch controller.whichDM
        case '1'
            if estimator.CL == 1 % if estimator is estimating closed loop or open loop field
                DM1command = DM1command + command + command_dither(1:DM.activeActNum);
            else
                %             DM1command = command + command_dither(1:DM.activeActNum); % for open loop estimation case
                
                DM1command = DM1command_DH + command + command_dither(1:DM.activeActNum); % for open loop estimation case
            end
            data.DMcommand(:, itr) = [DM1command; DM2command_DH]; % unused DM gets dark hole command
            
            command_dither(DM.activeActNum + 1 : end) = 0; %set unused mirror commands to zero
        case '2'
            if estimator.CL == 1
                DM2command = DM2command + command + command_dither(DM.activeActNum + 1 : end); %%
            else
                %             DM2command = command + command_dither(DM.activeActNum + 1 : end); % for open loop estimation case
                
                DM2command = DM2command_DH + command + command_dither(DM.activeActNum + 1 : end); % for open loop estimation case
            end
            
            data.DMcommand(:, itr) = [DM1command_DH;DM2command]; % unused DM gets dark hole command
            
            command_dither(1:DM.activeActNum) = 0;
        case 'both'
            %             DM1command = DM1command + command(1:DM.activeActNum) + command_dither(1:DM.activeActNum);
            %             DM2command = DM2command + command(DM.activeActNum + 1 : end) + command_dither(DM.activeActNum + 1 : end);
            %
            DM1command = DM1command_DH + command(1:DM.activeActNum) + command_dither(1:DM.activeActNum);
            DM2command = DM2command_DH + command(DM.activeActNum + 1 : end) + command_dither(DM.activeActNum + 1 : end);
            
            if itr>1
                data.DMcommand(:, itr) = [DM1command; DM2command] - data.Driftcommand(:,itr-1) ;
            end
            
        otherwise
            disp('You can only use the first DM, second DM or both for wavefront control.');
            return;
    end
    data.Dithercommand(:,itr) = command_dither;
    
    %% Introducing Drift
    % may need to edit first couple lines here if different coronograph is used
    marginWidth = (coronagraph.SPwidth - DM.widthDM)/2;
    marginNpixel = round(marginWidth / coronagraph.SPwidth * DM.DMmesh(1));
    %
    surf_drift = zeros(DM.DMmesh);
    surf_drift(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = ...
        target.driftDisp*(rand([DM.DMmesh - 2 * marginNpixel,1])-0.5);
    
    %     command_drift = 0.05*height2voltage(surf_drift, DM, target.driftDM, 5); %OP2, PAIR WITH DITHER OP2
    
    command_drift = target.driftScaling*height2voltage(surf_drift, DM, target.driftDM, 5); %used for 300 iter case initially
    
    %     command_drift = 0.01*height2voltage(surf_drift, DM, target.driftDM, 5); %OP2, PAIR WITH DITHER OP2
    %     command_drift = 0.05*height2voltage(surf_drift, DM, target.driftDM, 5); %OP1, PAIR WITH DITHER OP1
    
    %     command_drift = zeros(DM.activeActNum,1);
    estimator.driftStd = mean(abs(command_drift));
    switch target.driftDM % determine DM that will introduce drift and update command, this command DOES NOT get stored in "command"
        case '1'
            DM1command = DM1command + command_drift;% command; needs to be random
            data.Driftcommand(:,itr) = [command_drift; zeros(size(DM1command))];
            
            %             data.DMcommand(:, itr) =data.DMcommand(:, itr) - ...
            %                 [sum(data.Driftcommand(1:DM.activeActNum,1:itr-1), 2); zeros(DM.activeActNum,1)];
        case '2'
            DM2command = DM2command + command_drift;% command;
            data.Driftcommand(:,itr) = [zeros(size(DM2command));command_drift];
            
            %             data.DMcommand(:, itr) =data.DMcommand(:, itr) - ...
            %                 [ zeros(DM.activeActNum,1);sum(data.Driftcommand(DM.activeActNum + 1 : end,1:itr-1), 2)];
            
        otherwise
            disp('You can only use the first DM or second DM for speckle drift.');
            return;
    end
    
    %% for simulation, calculate the perfect contrast and E field
    if strcmpi(simOrLab, 'simulation')
        if target.broadBandControl
        else
            [EfocalStar, EfocalPlanet, I0] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
            data.Efocaltrue(:,itr) = EfocalStar(darkHole.pixelIndex);
            data.Iincotrue(:,itr) = abs(EfocalPlanet(darkHole.pixelIndex)).^2; % We can have perfect knowledge of the electric field in simulation
            
            if itr == 1
                contrastPerfect = zeros(Nitr, 1);
            end
            [EfocalStarNoise, EfocalPlanetNoise, InoNoise] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
            contrastPerfect(itr) = mean(InoNoise(darkHole.pixelIndex));
            data.EfocalPerfect(:, itr) = EfocalStarNoise(darkHole.pixelIndex);
            %             data.Efocaltrue(:,itr) = EfocalStar(darkHole.pixelIndex);
            data.Iincotrue(:,itr) = abs(EfocalPlanet(darkHole.pixelIndex)).^2; % We can have perfect knowledge of the electric field in simulation
            
        end
    end
    %% Open Loop Calculation
    switch target.driftDM
        case '1'
            if ItrImgOL(ItrImgOL == itr)
                itrOL = itrOL+1;
                imageOL = getImg(target, DM, coronagraph, camera, ...
                    sum(data.Driftcommand(1:DM.activeActNum,1:itr), 2) + data_DH.DMcommand(1:DM.activeActNum,end),...
                    data_DH.DMcommand(DM.activeActNum + 1 : end,end), simOrLab);
                data.imageSetLiveOL(:,:,itrOL) = imageOL;
                data.measuredContrastAverageLiveOL(itrOL,1) = mean(imageOL(darkHole.pixelIndex));
                
            end
            if strcmpi(simOrLab, 'lab')
                EfocalStar_openloop = data.EfocalEst0 + ...
                    model.G1 * sum(data.Driftcommand(1:DM.activeActNum,1:itr),2);
                Iopenloop = abs(EfocalStar_openloop).^2;
                data.EfocalEstOpenLoop(:,itr) = EfocalStar_openloop;
                
            else
                [EfocalStar_openloop_full, EfocalPlanet_openloop, Iopenloop_full] = ...
                    opticalModel(target, DM, coronagraph, camera, ...
                    sum(data.Driftcommand(1:DM.activeActNum,1:itr), 2) + data_DH.DMcommand(1:DM.activeActNum,end), ...
                    data_DH.DMcommand(DM.activeActNum + 1 : end,end));
                EfocalStar_openloop = EfocalStar_openloop_full(darkHole.pixelIndex);
                Iopenloop = Iopenloop_full(darkHole.pixelIndex);
                data.EfocalPerfOpenLoop(:,itr) = EfocalStar_openloop;
            end
            
        case '2'
            EfocalStar_openloop = data.EfocalEst0+ model.G2 * sum(data.Driftcommand(DM.activeActNum+1:end,1:itr),2);
            Iopenloop = abs(EfocalStar_openloop).^2;
        otherwise
            disp('You can only use the first DM or second DM for speckle drift.');
            return;
    end
    
    data.estOpenLoopContrast(itr,1) = mean(Iopenloop);
    
    %% Estimate the electric field
    disp(['Running ', estimator.type, ' estimator ...']);
    
    % Monochromatic case
    switch lower(estimator.type)
        case 'perfect'
            assert(strcmpi(simOrLab, 'simulation'), 'The perfect estimation can only be used in simulation!');
            [EfocalStar, EfocalPlanet, I0] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
            EfocalEst = EfocalStar(darkHole.pixelIndex);
            IincoEst = abs(EfocalPlanet(darkHole.pixelIndex)).^2; % We can have perfect knowledge of the electric field in simulation
            
        case 'ekf_speckle'
            % ** Only for comparison, remove later **
            
            dataAlt.DMcommand = data.DMcommand;% - data.Dithercommand; %IS THIS RIGHT
            [imageProbed, uProbed, dataAlt] = takeProbingImages(contrastEst, target, DM, coronagraph, camera, darkHole, estimatorAlt, DM1command, DM2command, simOrLab, dataAlt);
            if estimatorAlt.savedata
                dataAlt.imageSet{itr} = imageProbed;
                dataAlt.probeSet{itr} = uProbed;
            end
            % **
            % Real calc
            image = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
            u = zeros(DM.activeActNum,estimator.NumImg);
            
            %             image = abs(EfocalStar).^2; % FOR TESTING ONLY ********
            
            if estimator.nonProbeImage
                if estimator.EKFincoherent
                    [EfocalEst, IincoEst, data] = EKF(u, image, darkHole, model, estimator, controller, data);
                else
                    [EfocalEst, IincoEst, data] = EKF4(u, image, darkHole, model, estimator, controller, data);
                end
            else
                if estimator.EKFincoherent
                    % ** Only for comparison, remove later **
                    [EfocalEstProbed, IincoEstProbed, dataAlt] = batch(uProbed, imageProbed, darkHole, model, estimatorAlt, dataAlt);
                    dataAlt.EfocalEst(:,itr) = EfocalEstProbed;
                    % **
                    
                    % Real calc
                    
                    [EfocalEst, IincoEst, data] = EKF2_basic(u, image, darkHole, model, estimator, controller, data);
                else
                    % ** Only for comparison, remove later **
                    %
                    [EfocalEstProbed, IincoEstProbed, dataAlt] = batch(uProbed, imageProbed, darkHole, model, estimatorAlt, dataAlt);
                    dataAlt.EfocalEst(:,itr) = EfocalEstProbed;
                    % **
                    
                    % Real calc
                    if estimator.CL == 1
                        [EfocalEst, IincoEst, data] = EKF3b(u, image, darkHole, model, estimator, controller, data);
                    else
                        [EfocalEst, IincoEst, data] = EKF3b_OL(u, image, darkHole, model, estimator, controller, data);
                    end
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
    
    % NEED TO CHANGE ALL OF THIS TO BE CLOSED LOOP E FIELD?
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
    disp(['The estimated average contrast in the dark holes is ', num2str(mean(contrastEst))]);
    
    %% check the contrast after giving new control commands
    
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
    
    %% Probed comparison storage
    data.EfocalEstProbed(:,itr) = EfocalEstProbed;
    
    %% *REMOVE?* focal plane estimations in log scale after giving control commands
    IincoEst2D = zeros(size(I));
    if target.broadBandControl
        IincoEst2D(darkHole.pixelIndex) = mean(data.IincoEst(:, :, itr), 2);
    else
        IincoEst2D(darkHole.pixelIndex) = IincoEst;
    end
    
    
    IcoEst2D = zeros(size(I));
    if target.broadBandControl
        IcoEst2D(darkHole.pixelIndex) = mean(abs(data.EfocalEst(:, :, itr)).^2, 2);
    else
        IcoEst2D(darkHole.pixelIndex) = abs(EfocalEst).^2;
    end
    
    %% measured change of focal plane image
    if ~target.broadBandControl
        if itr == 1
            dImeasured = data.I(:,:,itr) - data.I0;
        else
            dImeasured = data.I(:,:,itr) - data.I(:,:,itr - 1);
        end
        dImeasured2D = zeros(size(dImeasured));
        dImeasured2D(darkHole.pixelIndex) = dImeasured(darkHole.pixelIndex);
        
        % linear predicted change of focal plane image * SHOULD INCLUDE
        % DITHER?*
        switch controller.whichDM
            case '1'
                dEmodel = model.G1 * (command + command_dither(1:DM.activeActNum));
            case '2'
                dEmodel = model.G2 * (command + command_dither(DM.activeActNum + 1 : end)); %*******
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
    end
    
    %% Open Loop E-field estimation when EKF uses CL field
    if estimator.CL == 1
        if strcmpi(simOrLab, 'simulation')
            data.EfocalEstOpenLoop(:,itr) = data.EfocalEst0 + model.G1*(sum(data.Driftcommand(1 : DM.activeActNum,1:itr),2));
        end
        if itr > 1
            EhatCL_10_full = [real(data.EfocalEst(:,itr-1)); imag(data.EfocalEst(:,itr-1))] + ...
                G * (data.efcCommand(:,itr) +data.Dithercommand(DM.activeActNum + 1 :end, itr)); %FOR DM2 AS DITHER DM
            EhatCL_10 = EhatCL_10_full(1:darkHole.pixelNum,1); % [real;imag]
            
            IhatCL10 = abs(EhatCL_10).^2;
            ICL10 = image(darkHole.pixelIndex);
            
            dEmodel_CL = model.G2 * (sum(data.efcCommand(:,1:itr),2) + sum(data.Dithercommand(DM.activeActNum + 1 : end,1:itr),2));
            
            EhatOL_10_viaCL = data.EfocalEst(:,itr) - dEmodel_CL; % Current iterations closed loop estimate minus total command applied to DM
            EhatOL_10_viaProbedCL = dataAlt.EfocalEst(:,itr) - dEmodel_CL;
            EhatOL_10_viaE0Drift = data.EfocalEstOpenLoop(:,itr);
            
            
            EhatCL_10_viaOL = data.EfocalPerfOpenLoop(:,itr) + dEmodel_CL;
            
            IhatCL_10_viaOL = abs(EhatCL_10_viaOL).^2;
            
            EhatOL_err_viaCL = abs((real(data.EfocalPerfOpenLoop(:,itr)) - ...
                real(EhatOL_10_viaCL))./real(data.EfocalPerfOpenLoop(:,itr)));
            EhatOL_err_viaE0Drift = abs((real(data.EfocalPerfOpenLoop(:,itr)) - ...
                real(data.EfocalEstOpenLoop(:,itr)))./real(data.EfocalPerfOpenLoop(:,itr)));
        end
    end
    %% Write data for post-processing
    %% Visualizations
    % focal plane estimations in log scale after giving control commands
    Plot_SpeckleMaint
    
    
end
%% save data
eval([data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), '=data;']);
cd(folder.dataLibrary);
eval(['save ', data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), ' ', data.controllerType, coronagraph.type, num2str(yyyymmdd(datetime('today'))), 'Trial', num2str(runTrial), ';']);
cd(folder.main);

%% Get Prediction of OL
DM1command = data_DH.DMcommand(1:DM.activeActNum,end);
DM2command = data_DH.DMcommand(DM.activeActNum + 1 : end,end);
for itr = 1:Nitr
    if itr <= 3
        camera.exposure = 0.5;%0.01;
        camera.exposure0 = 0.5;%0.01;
    elseif itr <= 15
        camera.exposure = 0.5;
        camera.exposure0 = 0.5;
    else
        camera.exposure = 0.5;
        camera.exposure0 = 0.5;
    end
    DM1command = DM1command + data.Driftcommand(1 : DM.activeActNum,itr);
    image = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
end

%% Re-plot data

for itr = 1:Nitr
    %    dEmodel = model.G2 * (data.DMcommand(DM.activeActNum + 1 : end,itr) + data.Dithercommand(DM.activeActNum + 1 : end,itr));
    %    EfocalEstNew = data.EfocalEst(:,itr) + dEmodel;
    %    data.estimatedContrastAverage(itr) = ;
    %     Plot_SpeckleMaint
    if target.broadBandControl
        figure(1), imagesc(log10(abs(mean(data.I(:, :, :, itr), 3)))), colorbar
    else
        figure(1), imagesc(log10(abs(data.I(:,:,itr)))), colorbar;
    end
    caxis(cRange);
    title(['After control iteration ', num2str(itr)]);
    drawnow
end


%%
EhatOL_err_viaCL_mean = zeros(Nitr,1);
EhatOL_err_viaE0Drift_mean = zeros(Nitr,1);
IhatCL_viaOL_mean = zeros(Nitr,1);
for itr = 1:Nitr
    data.EfocalEstOpenLoop(:,itr) = data.EfocalEst0 + model.G1*(sum(data.Driftcommand(1 : DM.activeActNum,1:itr),2));
    EhatOL_err_viaCL_mean(itr,1) = mean(abs(real(data.EfocalPerfOpenLoop(:,itr)) - ...
        real(EhatOL_10_viaCL)))./mean(abs(real(data.EfocalPerfOpenLoop(:,itr))));
    EhatOL_err_viaE0Drift_mean(itr,1) = mean(abs(real(data.EfocalPerfOpenLoop(:,itr)) - ...
        real(data.EfocalEstOpenLoop(:,itr))))./mean(abs(real(data.EfocalPerfOpenLoop(:,itr))));
    
    dEmodel_CL = model.G2 * (sum(data.efcCommand(:,1:itr),2) + sum(data.Dithercommand(DM.activeActNum + 1 : end,1:itr),2));
    EhatCL_10_viaOL = data.EfocalEstOpenLoop(:,itr) + dEmodel_CL;
    IhatCL_viaOL_mean(itr,1) = mean(abs(EhatCL_10_viaOL).^2);
    
end

figure;
plot(1:Nitr,EhatOL_err_viaCL_mean ,1:Nitr, EhatOL_err_viaE0Drift_mean)
legend('$mean(|\hat{E}^{OLviaCL}_{i} - E^{OL}|)/mean(|E^{OL}|) $',...
    '$mean(|\hat{E}^{OLviaUdrift}_{i} - E^{OL}|)/mean(|E^{OL}|) $','interpreter','latex')

figure;
plot(1:Nitr,data.measuredContrastAverage,1:Nitr,data.estimatedContrastAverage,...
    1:Nitr,IhatCL_viaOL_mean)
legend('$\bar{I}^{measured}$','$\bar{\hat{I}}^{CL}$',...
    '$mean(|\hat{E}^{OL}_i + G\sum_{1}^{i}(u^{EFC}_i + u^{dith}_i)|^2)$',...
    'interpreter','latex','fontsize',14)
