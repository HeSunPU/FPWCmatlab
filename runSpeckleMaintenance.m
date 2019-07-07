%% Set number of iterations

Nitr = 5;
%% Load dark hole DM commands

% load command
DM1command = data.DMcommand(:,1);
DM2command = data.DMcommand(:,2);

data_DH = data;
clear data
% save drift DM command as a different variable
DM1command_DH = DM1command;
DM2command_DH = DM2command;


%% Set up dither and drift

target.driftcmd
estimator.dither

drift = drift + 2*2*pi*1e-11/target.starWavelength*(numpy.random.random(drift.shape)-0.5) ;% in nm

command_dither = normrnd(0,target.driftStd,[DM.activeActNum,1]); %THIS IS DITHER COMMAND

dus.append(numpy.random.normal(0, sigma_u, u.shape)) %DITHER

%% Control loop start
for itr = 1 : Nitr
    if itr <= 3
        camera.exposure = 0.01;
        camera.exposure0 = 0.01;
    elseif itr <= 15
        camera.exposure = 0.1;
        camera.exposure0 = 0.1;
    else
        camera.exposure = 0.3;
        camera.exposure0 = 0.3;
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
                G = [real(G); imag(G)]; %%
                x = [real(EfocalEst); imag(EfocalEst)];
                if controller.adaptiveEFC % automatically choose the regularization parameter
                    controller = adaptiveEFC(x, G, target, DM, coronagraph, camera, darkHole, controller, DM1command, DM2command, simOrLab);
                    disp(['Best alpha: ', num2str(controller.alpha)])
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
    
%% Introducing Drift
    % may need to edit first couple lines here if different coronograph is used 
    marginWidth = (coronagraph.SPwidth - DM.widthDM)/2;
    marginNpixel = round(marginWidth / coronagraph.SPwidth * DM.DMmesh(1));
    
    surf_drift = zeros(DM.DMmesh);
    surf_drift(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = ...
        surf_disp*(rand([DM.DMmesh - 2 * marginNpixel,1])-0.5);
        
    command_drift = height2voltage(surf_drift, DM, target.driftDM, 5);

    switch target.driftDM % determine DM that will introduce drift and update command, this command DOES NOT get stored in "command"
        case '1'
            DM1command = DM1command + command_drift;% command; needs to be random
            data.Driftcommand(:,iter) = [DM1command; zeros(size(DM1command))];
        case '2'
            DM2command = DM2command + command_drift;% command;
            data.Driftcommand(:,iter) = [zeros(size(DM2command));DM2command];
        otherwise
            disp('You can only use the first DM or second DM for speckle drift.');
            return;
    end
    
    %% Susan edited this block a bit    
    command_dither = [normrnd(0,estimator.ditherStd,[DM.activeActNum,1]);...
        normrnd(0,estimator.ditherStd,[DM.activeActNum,1])];%THIS IS DITHER COMMAND, check that it is within the resolution of the system
    
    switch controller.whichDM
        case '1'
            DM1command = DM1command + command + command_dither(1:DM.activeActNum);
            
            data.DMcommand(:, itr) = [DM1command; zeros(size(DM1command))];
            command_dither(DM.activeActNum + 1 : end) = 0; %set unused mirror commands to zero
        case '2'
            DM2command = DM2command + command + command_dither(DM.activeActNum + 1 : end); %%
            
            data.DMcommand(:, itr) = [zeros(size(DM1command));DM2command];
            command_dither(1:DM.activeActNum) = 0;
        case 'both'
            DM1command = DM1command + command(1:DM.activeActNum) + command_dither(1:DM.activeActNum);
            DM2command = DM2command + command(DM.activeActNum + 1 : end) + command_dither(DM.activeActNum + 1 : end);
            
            data.DMcommand(:, itr) = [DM1command; DM2command];
        otherwise
            disp('You can only use the first DM, second DM or both for wavefront control.');
            return;
    end
    data.Dithercommand(:,iter) = command_dither;  
    
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
    
    % Non-broadband case
    switch lower(estimator.type)
        case 'perfect'
            assert(strcmpi(simOrLab, 'simulation'), 'The perfect estimation can only be used in simulation!');
            [EfocalStar, EfocalPlanet, I0] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
            EfocalEst = EfocalStar(darkHole.pixelIndex);
            IincoEst = abs(EfocalPlanet(darkHole.pixelIndex)).^2; % We can have perfect knowledge of the electric field in simulation
        case 'ekf_speckle'
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
