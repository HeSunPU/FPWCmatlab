%% Visualizations

% open loop estimation


% NEED TO SEND CONTROL DM FLAT MAP AS DM COMMAND HERE?
% unused drift DM should send dark hole command
switch target.driftDM
    case '1'
        [EfocalStar_openloop, EfocalPlanet_openloop, Iopenloop] = ...
            opticalModel(target, DM, coronagraph, camera, ...
            sum(data.Driftcommand(1:DM.activeActNum,1:itr), 2) + data_DH.DMcommand(1:DM.activeActNum,end), ...
            data_DH.DMcommand(DM.activeActNum + 1 : end,end));
        
    case '2'
        [EfocalStar_openloop, EfocalPlanet_openloop, Iopenloop] = ...
            opticalModel(target, DM, coronagraph, camera, ...
            data_DH.Driftcommand(1:DM.activeActNum,end),  ...
            data.Driftcommand(DM.activeActNum + 1 : end,itr)+ data_DH.DMcommand(DM.activeActNum + 1 : end,end));     
    otherwise
        disp('You can only use the first DM or second DM for speckle drift.');
        return;
end

data.estOpenLoopContrast(itr,1) = mean(Iopenloop(darkHole.pixelIndex));


% focal plane estimations in log scale after giving control commands
IincoEst2D = zeros(size(I));
if target.broadBandControl
    IincoEst2D(darkHole.pixelIndex) = mean(data.IincoEst(:, :, itr), 2);
else
    IincoEst2D(darkHole.pixelIndex) = IincoEst;
end
% figure(10), imagesc(log10(abs(IincoEst2D))), colorbar;
% caxis(cRange);
% title(['Incoherent light after control iteration ', num2str(itr)]);
% drawnow

IcoEst2D = zeros(size(I));
if target.broadBandControl
    IcoEst2D(darkHole.pixelIndex) = mean(abs(data.EfocalEst(:, :, itr)).^2, 2);
else
    IcoEst2D(darkHole.pixelIndex) = abs(EfocalEst).^2;
end
% figure(11), imagesc(log10(abs(IcoEst2D))), colorbar;
% caxis(cRange);
% title(['Coherent light after control iteration ', num2str(itr)]);
% drawnow

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
xlabel('iteration');
ylabel('contrast');
drawnow

figure(24); plot(1:DM.activeActNum,command,'r',...
    1:2*DM.activeActNum,data.Dithercommand(:, itr),'b',...
     1:2*DM.activeActNum,data.Driftcommand(:,itr),'g');
axis tight
legend('DM Control Command','Dither Command','Drift Command');
drawnow

figure(42); plot(1:darkHole.pixelNum,real(data.EfocalPerfect(:,itr)),'r',...
    1:darkHole.pixelNum,real(data.EfocalEst(:,itr)),'b');
axis tight
legend('True','Estimated');
title('True vs Estimated E Field');

drawnow

%%
% figure(43); 
% subplot(1,2,1)
% plot(1:darkHole.pixelNum,abs(real(data.EfocalEst(:,:))-real(data.Efocaltrue(:,:))));
% axis tight
% title('Estimated - True E Field');
% 
% subplot(1,2,2)
% plot(1:darkHole.pixelNum,abs(real(data.EfocalEst(:,:))+real(data.Efocaltrue(:,:))));
% axis tight
% title('Estimated + True E Field');
%%

% figure(42); plot(1:darkHole.pixelNum,real(data_DH.EfocalPerfect(:, end)),'r',...
%     1:darkHole.pixelNum,real(data.EfocalEst0),'b');
% axis tight
% legend('Perfect end DH E','End Estimated E');
% title('Perfect vs Estimated E Field at end of DH dig');
%%
% 
% figure(43); 
% subplot(2,2,1)
% plot(1:darkHole.pixelNum,real(data.EfocalPerfect(:,1)),'r',...
%     1:darkHole.pixelNum,real(data.EfocalEst0),'b');
% axis tight
% legend('True','Estimated');
% title('Iteration1');
% 
% subplot(2,2,2)
% plot(1:darkHole.pixelNum,real(data.EfocalPerfect(:,2)),'r',...
%     1:darkHole.pixelNum,real(data.EfocalEst(:,1)),'b');
% axis tight
% legend('True','Estimated');
% title('Iteration2');
% 
% subplot(2,2,3)
% plot(1:darkHole.pixelNum,real(data.EfocalPerfect(:,3)),'r',...
%     1:darkHole.pixelNum,real(data.EfocalEst(:,2)),'b');
% axis tight
% legend('True','Estimated');
% title('Iteration3');
% 
% subplot(2,2,4)
% plot(1:darkHole.pixelNum,real(data.EfocalPerfect(:,4)),'r',...
%     1:darkHole.pixelNum,real(data.EfocalEst(:,3)),'b');
% axis tight
% legend('True','Estimated');
% title('Iteration4');

%%
% EhatCL_1_full = [real(data.EfocalEst0); imag(data.EfocalEst0)] + G * (command +command_dither(DM.activeActNum + 1 : end)); %FOR DM2 AS DITHER DM
% EhatCL_1 = EhatCL_1_full(1:darkHole.pixelNum,1); % [real;imag]
% 
% IhatCL1 = abs(EhatCL_1).^2;
% ICL1 = image(darkHole.pixelIndex);
% 
% figure(43); 
% 
% subplot(2,2,1)
% plot(1:darkHole.pixelNum,real(data_DH.EfocalEst(:,end)),'r',...
%     1:darkHole.pixelNum,real(data.EfocalEst0),'b');
% axis tight
% legend('EOL_0','EhatOL_0');
% title('Step 0 Open Loop');
% 
% subplot(2,2,2)
% plot(1:darkHole.pixelNum,real(data.EfocalPerfect(:,1)),'r',...
%     1:darkHole.pixelNum,EhatCL_1,'b'); % IS THIS ONE RIGHT
% axis tight
% legend('ECL_1','EhatCL_1 (post command and dither)');
% title('Step 1 Closed Loop E field');
% 
% subplot(2,2,3)
% plot(1:darkHole.pixelNum,real(ICL1),'r',...
%     1:darkHole.pixelNum,IhatCL1,'b');
% axis tight
% legend('ICL_1','IhatCL_1');
% title('Step 1 Closed Loop Intensity');
% 
% subplot(2,2,4)
% plot(1:darkHole.pixelNum,real(data.EfocalPerfect(:,1)),'r',...
%     1:darkHole.pixelNum,real(data.EfocalEst(:,1)),'b');
% axis tight
% legend('EOL_1','EhatOL_1');
% title('Step 1 Open Loop E Field');






%%

% if (strcmpi(simOrLab, 'simulation'))
%     if target.broadBandControl
%         figure(22), semilogy(0:itr, mean([data.contrast0, contrastPerfect(:, 1:itr)], 1), '-o');
%     else
%         figure(22), semilogy(0:itr, [data.contrast0; contrastPerfect(1:itr)], '-o');
%     end
%     ylim([10^(cRange(1)), 10^(cRange(2))]);
%     legend('perfect');
%     drawnow
% end

% measured change of focal plane image
if ~target.broadBandControl
    if itr == 1
        dImeasured = data.I(:,:,itr) - data.I0;
    else
        dImeasured = data.I(:,:,itr) - data.I(:,:,itr - 1);
    end
%     dImeasured2D = zeros(size(dImeasured));
%     dImeasured2D(darkHole.pixelIndex) = dImeasured(darkHole.pixelIndex);
%     figure(3), imagesc(log10(abs(dImeasured2D))), colorbar;
%     title('Measured Intensity Change');
%     caxis(cRange);
%     drawnow
%     
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
%     
%     figure(4), imagesc(log10(abs(dImodel2D))), colorbar;
%     title('Model-predicted Intensity Change');
%     caxis(cRange);
%     drawnow
end