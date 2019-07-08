%% Visualizations

% open loop estimation


% NEED TO SEND CONTROL DM FLAT MAP AS DM COMMAND HERE?
[EfocalStar_openloop, EfocalPlanet_openloop, Iopenloop] = ...
    opticalModel(target, DM, coronagraph, camera, data.Driftcommand(1:DM.activeActNum,itr), data.Driftcommand(DM.activeActNum + 1 : end,itr));

data.estOpenLoopContrast(itr,1) = mean(Iopenloop(darkHole.pixelIndex));


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
    figure(2), semilogy(0:itr, [data.contrast0; data.measuredContrastAverage(1:itr)],'-o',...
        0:itr, [data.estimatedContrastAverage0; data.estimatedContrastAverage(1:itr)],'-s', ...
        0:itr, [data.estimatedIncoherentAverage0; data.estimatedIncoherentAverage(1:itr)], '-^',...
        0:itr,[data.estimatedContrastAverage0; data.estOpenLoopContrast(1:itr)],'-d');
end
% ylim([10^(cRange(1)), 10^(cRange(2))]);
ylim([10^(cRange(1)), 10^(cRange(2)+2)]);
legend('measured', 'estimated', 'incoherent','open loop');
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