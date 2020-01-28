%% Visualizations used for running experiment

%% focal plane images given control commands in log scale
if target.broadBandControl
    figure(1), imagesc(log10(abs(mean(data.I(:, :, :, itr), 3)))), colorbar
else
    figure(1), subplot(2,3,1), imagesc(log10(abs(data.I(:,:,itr)))), colorbar;
end
caxis(cRange);
title(['After control iteration ', num2str(itr)]);

subplot(2,3,2), imagesc(log10(abs(dImeasured2D))), colorbar;
title('Measured Intensity Change');
caxis(cRange);
subplot(2,3,5), imagesc(log10(abs(dImodel2D))), colorbar;
title('Model-predicted Intensity Change');
caxis(cRange);
    
subplot(2,3,3), imagesc(log10(abs(IincoEst2D))), colorbar;
caxis(cRange);
title(['Incoherent light after control iteration ', num2str(itr)]);

subplot(2,3,6), imagesc(log10(abs(IcoEst2D))), colorbar;
caxis(cRange);
title(['Coherent light after control iteration ', num2str(itr)]);

%%
% figure;
% semilogy(0:itr, [data.contrast0; data.measuredContrastAverage(1:itr)],'-o',...
%         0:itr, [data.estimatedContrastAverage0; data.estimatedContrastAverage(1:itr)],'-s', ...
%         0:itr,[data.estimatedContrastAverage0; data.estOpenLoopContrast(1:itr)],'-d',...
%         ItrImgOL(1:itrOL),data.measuredContrastAverageLiveOL(1:itrOL,1),'-p');
% ylim([10^(cRange(1)), 10^(cRange(2)+2)]);
% legend('measured', 'estimated','open loop est','open loop measured');
% xlabel('iteration');
% ylabel('contrast');
% drawnow
%% contrast correction curve - average
if target.broadBandControl
    %     figure(2), semilogy(0:itr, mean([data.contrast0, data.measuredContrastAverage(:, 1:itr)], 1), '-o' ,0:itr, mean([data.estimatedContrastAverage0, data.estimatedContrastAverage(:, 1:itr)], 1), '-s', 0:itr, mean([data.estimatedIncoherentAverage0, data.estimatedIncoherentAverage(:, 1:itr)], 1), '-^');
else
    
    subplot(2,3,4), semilogy(0:itr, [data.contrast0; data.measuredContrastAverage(1:itr)],'-o',...
        0:itr, [data.estimatedContrastAverage0; data.estimatedContrastAverage(1:itr)],'-s', ...
        0:itr, [data.estimatedIncoherentAverage0; data.estimatedIncoherentAverage(1:itr)], '-^',...
        0:itr,[data.estimatedContrastAverage0; data.estOpenLoopContrast(1:itr)],'-d',...
        ItrImgOL(1:itrOL),data.measuredContrastAverageLiveOL(1:itrOL,1),'-p');
end
% ylim([10^(cRange(1)), 10^(cRange(2))]);
ylim([10^(cRange(1)), 10^(cRange(2)+2)]);
legend('measured', 'estimated', 'incoherent','open loop est','open loop measured');
xlabel('iteration');
ylabel('contrast');
drawnow

figure(24); plot(DM.activeActNum+1:2*DM.activeActNum,data.efcCommand(:,itr),'r',...
    1:2*DM.activeActNum,data.Dithercommand(:, itr),'b',...
    1:2*DM.activeActNum,data.Driftcommand(:,itr),'g');
axis tight
legend('DM Control Command','Dither Command','Drift Command');
drawnow

%% Probed E field estimation vs unprobed E field estimation
if estimator.CL == 1 && strcmpi(simOrLab, 'simulation')
    figure(7)
    subplot(1,2,1)
    plot(1:darkHole.pixelNum,real(data.EfocalEst(:,itr)),'r',...
        1:darkHole.pixelNum,real(data.EfocalEstProbed(:,itr)),'b',...
        1:darkHole.pixelNum,real(data.EfocalPerfect(:,itr)),'g');
    
    % plot(1:darkHole.pixelNum,real(data.EfocalEst(:,itr)),'r',...
    %     1:darkHole.pixelNum,real(data.EfocalPerfect(:,itr)),'g');
    axis tight
    legend('Dither','Probed','Perfect');
    xlabel('iteration')
    ylabel('Estimated Electric Field')
    title({'Comparison bt probed E estimation and unprobed E estimation (unprobed used for control)',' '})
    %
    EfocalEst_err(itr,1) = mean(abs( (real(dataAlt.EfocalEst(:,itr)) - real(data.EfocalEst(:,itr)))))...
        ./ mean(abs(real(dataAlt.EfocalEst(:,itr)) ));
    
    EfocalEstPerf_err(itr,1) = mean(abs( (real(data.EfocalPerfect(:,itr)) - real(data.EfocalEst(:,itr)))))...
        ./ mean(abs(real(data.EfocalPerfect(:,itr)) ));
    
    EfocalEstProbed_err(itr,1) = mean(abs( (real(data.EfocalPerfect(:,itr)) - real(dataAlt.EfocalEst(:,itr)))))...
        ./ mean(abs(real(data.EfocalPerfect(:,itr)) ));
    
    subplot(1,2,2)
    plot(1:itr,EfocalEst_err(1:itr),'r+-',1:itr,EfocalEstPerf_err(1:itr),'b+-',1:itr,EfocalEstProbed_err(1:itr),'g+-');
    % plot(1:itr,EfocalEstPerf_err(1:itr),'b+-');
    axis tight
    xlabel('iteration')
    ylabel('Average |Relative Error|')
    title({'Average Relative Error bt Probed and Unprobed E-Field Estimation',' '})
    legend('Dither-Probed','Dither-Perfect','Probed-Perfect')
    ylim([0,1])
    drawnow
end
if estimator.CL == 0 && strcmpi(simOrLab, 'simulation')
    figure(7)
    subplot(1,2,1)
    plot(1:darkHole.pixelNum,real(data.EfocalEst(:,itr)),'r',...
        1:darkHole.pixelNum,real(data.EfocalPerfOpenLoop(:,itr)),'g');
    
    % plot(1:darkHole.pixelNum,real(data.EfocalEst(:,itr)),'r',...
    %     1:darkHole.pixelNum,real(data.EfocalPerfect(:,itr)),'g');
    axis tight
    legend('Dither','Perfect OL');
    xlabel('iteration')
    ylabel('Estimated OL Electric Field')
    title({'Comparison bt probed E estimation and unprobed E estimation (unprobed used for control)',' '})
    %
%     EfocalEst_err(itr,1) = mean(abs( (real(dataAlt.EfocalEst(:,itr)) - real(data.EfocalEst(:,itr)))))...
%         ./ mean(abs(real(dataAlt.EfocalEst(:,itr)) ));
    
    EfocalEstPerf_err_tot(:,itr) = (abs( (real(data.EfocalPerfOpenLoop(:,itr)) - real(data.EfocalEst(:,itr)))))...
        ./ mean(abs(real(data.EfocalPerfOpenLoop(:,itr)) ));
    
    EfocalEstPerf_err(itr,1) = mean(EfocalEstPerf_err_tot(:,itr));
    
%     EfocalEstProbed_err(itr,1) = mean(abs( (real(data.EfocalEstOpenLoop(:,itr)) - real(dataAlt.EfocalEst(:,itr)))))...
%         ./ mean(abs(real(data.EfocalEstOpenLoop(:,itr)) ));
    
    subplot(1,2,2)
    plot(1:itr,EfocalEstPerf_err(1:itr),'b+-');
    % plot(1:itr,EfocalEstPerf_err(1:itr),'b+-');
    axis tight
    xlabel('iteration')
    ylabel('Average |Relative Error|')
    title({'Average Relative Error bt Perfect and Estimated OL E-Field',' '})
    legend('Dither-Perfect')
    ylim([0,1])
    drawnow
    
    figure(77);
    plot(EfocalEstPerf_err_tot(:,itr))
    title('EKF Estimate -  OL Model Calc')
    axis tight
    drawnow
end

if strcmpi(simOrLab, 'lab')
    figure(7)
    subplot(1,2,1)
    plot(1:darkHole.pixelNum,real(data.EfocalEst(:,itr)),'r',...
        1:darkHole.pixelNum,real(dataAlt.EfocalEst(:,itr)),'g');
    
    % plot(1:darkHole.pixelNum,real(data.EfocalEst(:,itr)),'r',...
    %     1:darkHole.pixelNum,real(data.EfocalPerfect(:,itr)),'g');
    axis tight
    legend('Dither','Batch');
    xlabel('iteration')
    ylabel('Estimated CL Electric Field')
    title({'Comparison bt probed E estimation and \\ unprobed E estimation (unprobed used for control)',' '})
   
    %
%     EfocalEst_err(itr,1) = mean(abs( (real(dataAlt.EfocalEst(:,itr)) - real(data.EfocalEst(:,itr)))))...
%         ./ mean(abs(real(dataAlt.EfocalEst(:,itr)) ));
    
    EfocalEstAlt_err_tot(:,itr) = (abs( (real(dataAlt.EfocalEst(:,itr)) - real(data.EfocalEst(:,itr)))))...
        ./ mean(abs(real(dataAlt.EfocalEst(:,itr)) ));
    
    EfocalEstAlt_err(itr,1) = mean(EfocalEstAlt_err_tot(:,itr));
    
%     EfocalEstProbed_err(itr,1) = mean(abs( (real(data.EfocalEstOpenLoop(:,itr)) - real(dataAlt.EfocalEst(:,itr)))))...
%         ./ mean(abs(real(data.EfocalEstOpenLoop(:,itr)) ));
    
    subplot(1,2,2)
    plot(1:itr,EfocalEstAlt_err(1:itr),'b+-');
    % plot(1:itr,EfocalEstPerf_err(1:itr),'b+-');
    axis tight
    xlabel('iteration')
    ylabel('Average |Relative Error|')
    title({'Average Relative Error bt Perfect and Estimated OL E-Field',' '})
    legend('Dither-Perfect')
    ylim([0,1])
    drawnow
 
    
    figure(77);
    plot(EfocalEstAlt_err_tot(:,itr))
    title('EKF Estimate -   Batch Estimate')
    axis tight
    drawnow
end

%% measured and modelled change of focal plane image



 %     dImeasured2D = zeros(size(dImeasured));
%     dImeasured2D(darkHole.pixelIndex) = dImeasured(darkHole.pixelIndex);
%     figure(3), imagesc(log10(abs(dImeasured2D))), colorbar;
%     title('Measured Intensity Change');
%     caxis(cRange);
%     drawnow
%     

%     
%     figure(4), imagesc(log10(abs(dImodel2D))), colorbar;
%     title('Model-predicted Intensity Change');
%     caxis(cRange);
%     drawnow


%% random
% 
%  temp = real(data.EfocalEst(:,itr)) + mean(real(data.EfocalPerfect(:, itr)));
% temp_err = mean(abs( (real(data.EfocalPerfect(:,itr)) - temp)))...
%     ./ mean(abs(real(data.EfocalPerfect(:,itr)) ));


% EhatCL_1_full = [real(data.EfocalEst0); imag(data.EfocalEst0)] + G * (command +command_dither(DM.activeActNum + 1 : end)); %FOR DM2 AS DITHER DM
% EhatCL_1 = EhatCL_1_full(1:darkHole.pixelNum,1); % [real;imag]
% 
% IhatCL1 = abs(EhatCL_1).^2;
% ICL1 = image(darkHole.pixelIndex);
% 
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

%% Open loop estimation case
if itr >1 && estimator.CL == 0 && strcmpi(simOrLab, 'simulation')
    EhatCL_10_full = [real(data.EfocalEst(:,itr-1)); imag(data.EfocalEst(:,itr-1))] + ...
        G * (data.DMcommand(DM.activeActNum + 1 : end,itr)); %FOR DM2 AS DITHER DM
    EhatCL_10 = EhatCL_10_full(1:darkHole.pixelNum,1); % [real;imag]
    
    IhatCL10 = abs(EhatCL_10).^2;
    ICL10 = image(darkHole.pixelIndex);
    
    dEmodel_OL = model.G2 * (data.DMcommand(DM.activeActNum + 1 : end,itr));
    EhatCL_10_viaOL = data.EfocalEstOpenLoop(:,itr) + dEmodel_OL;
    IhatCL_10_viaOL = abs(EhatCL_10_viaOL).^2;
    
    
    figure(43);
    
    subplot(2,2,1)%wrong
    plot(1:darkHole.pixelNum,real(data.EfocalEstOpenLoop(:,itr-1)),'r',...
        1:darkHole.pixelNum,real(data.EfocalEst(:,itr-1)),'b');
    axis tight
    legend('EOL_{9}','EhatOL_{9}');
    title('Step 10 Initial Open Loop');
    
    subplot(2,2,2)
    plot(1:darkHole.pixelNum,real(data.EfocalPerfect(:,itr)),'r',...
        1:darkHole.pixelNum,EhatCL_10,'b'); % IS THIS ONE RIGHT
    axis tight
    legend('ECL_{10}','EhatCL_{10} (post command and dither)');
    title('Step 10 Closed Loop E field');
    
    % subplot(2,2,3)
    % plot(1:darkHole.pixelNum,real(ICL10),'r',...
    %     1:darkHole.pixelNum,IhatCL10,'b');
    % axis tight
    % legend('ICL_{10}','IhatCL_{10}');
    % title('Step 10 Closed Loop Intensity');
    subplot(2,2,3)
    plot(1:darkHole.pixelNum,real(ICL10),'r',...
        1:darkHole.pixelNum,IhatCL10,'b')%,1:darkHole.pixelNum,IhatCL_10_viaOL,'c');
    axis tight
    % legend('ICL_{10}','IhatCL_{10 via OL}');
    legend('ICL_{10}','IhatCL_{10}','IhatCL_{10 via OL}');
    title('Step 10 Closed Loop Intensity');
    
    subplot(2,2,4)
    plot(1:darkHole.pixelNum,real(data.EfocalEstOpenLoop(:,itr)),'r',...
        1:darkHole.pixelNum,real(data.EfocalEst(:,itr)),'b');
    axis tight
    legend('EOL_{10}','EhatOL_{10}');
    title('Step 10 Open Loop E Field');
end

%%
if itr > 1 && estimator.CL == 1 && strcmpi(simOrLab, 'simulation')
    EhatCL_10_full = [real(data.EfocalEst(:,itr-1)); imag(data.EfocalEst(:,itr-1))] + ...
        G * (data.efcCommand(:,itr) +data.Dithercommand(DM.activeActNum + 1 :end, itr)); %FOR DM2 AS DITHER DM
    EhatCL_10 = EhatCL_10_full(1:darkHole.pixelNum,1); % [real;imag]
    
    IhatCL10 = abs(EhatCL_10).^2;
    ICL10 = image(darkHole.pixelIndex);
    
    dEmodel_CL = model.G2 * (sum(data.efcCommand(:,1:itr),2) + sum(data.Dithercommand(DM.activeActNum + 1 : end,1:itr),2));
    EhatCL_10_viaOL = data.EfocalEstOpenLoop(:,itr) + dEmodel_CL;
    
    EhatOL_10_viaCL = data.EfocalEst(:,itr) - dEmodel_CL; % Current iterations closed loop estimate minus total command applied to DM
    EhatOL_10_viaProbedCL = dataAlt.EfocalEst(:,itr) - dEmodel_CL;
    EhatOL_10_viaE0Drift = data.EfocalEst0 + model.G1*(sum(data.Driftcommand(1 : DM.activeActNum,1:itr),2));
        
    
    
    IhatCL_10_viaOL = abs(EhatCL_10_viaOL).^2;
    
    EhatOL_err_viaCL = abs((real(data.EfocalEstOpenLoop(:,itr)) - ...
        real(EhatOL_10_viaCL))./real(data.EfocalEstOpenLoop(:,itr)));
    EhatOL_err_viaE0Drift = abs((real(data.EfocalEstOpenLoop(:,itr)) - ...
        real(EhatOL_10_viaE0Drift))./real(data.EfocalEstOpenLoop(:,itr)));
    
    figure(34);
    
    subplot(2,2,1)%wrong
    plot(1:darkHole.pixelNum,real(data.EfocalPerfect(:,itr-1)),'r',...
        1:darkHole.pixelNum,real(data.EfocalEst(:,itr-1)),'b');
    axis tight
    legend('$E^{CL}_{i-1} = model$','$\hat{E}^{CL}_{i-1} =$ old EKF output','interpreter','latex');
    title('Step i-1 Closed Loop');
    
    subplot(2,2,2)
    plot(1:darkHole.pixelNum,real(data.EfocalPerfect(:,itr)),'r',...
        1:darkHole.pixelNum,EhatCL_10,'b'); % IS THIS ONE RIGHT
    axis tight
    legend('$E^{CL}_{i} = model$','$\hat{E}^{CL}_{i}|_{I^{CL}} =$ EKF output','interpreter','latex');
%     legend('ECL_{i}','EhatCL_{i} (post command and dither)','interpreter','latex');
    title('Step i Closed Loop E field');
    
    subplot(2,2,3)
    plot(1:darkHole.pixelNum,real(ICL10),'r',...
        1:darkHole.pixelNum,IhatCL10,'b',1:darkHole.pixelNum,IhatCL_10_viaOL,'c');
    axis tight
    % legend('ICL_{10}','IhatCL_{10 via OL}');
    legend('$I^{CL}_{i} = image $','$\hat{I}^{CL}_{i} = |\hat{E}^{CL}_i|^2$',...
        '$\hat{I}^{CL via OL}_{i} = |\hat{E}^{OL}_{i} + G(u^{EFC}_i+u^{dither}_i)|^2$','interpreter','latex');
    title('Step i Closed Loop Intensity');

    subplot(2,2,4)
    plot(1:darkHole.pixelNum,real(data.EfocalEstOpenLoop(:,itr)),'r',...
        1:darkHole.pixelNum,real(EhatOL_10_viaE0Drift),'r--',...
        1:darkHole.pixelNum,real(EhatOL_10_viaCL),'b',...
        1:darkHole.pixelNum, real(EhatOL_10_viaProbedCL),'g');
    axis tight
    legend('$E^{OL}_{i}$','$\hat{E}^{OL}_{i}|_{E^{OL}_0,u^{drift}}$','$\hat{E}^{OL}_{i}$',...
        '$\hat{E}^{OL}_{i}|_{I^{probe}}$','interpreter','latex');
    title('Step i Open Loop E Field');
    
    figure(44);
    plot(1:darkHole.pixelNum,EhatOL_err_viaE0Drift,'r--',...
        1:darkHole.pixelNum,EhatOL_err_viaCL,'b',...
        1:darkHole.pixelNum,EhatOL_err_viaE0Drift+ EhatOL_err_viaCL, 'g');
    xlabel('Pixel')
    ylabel('Relative Error')
    legend('OL Error via Drift','OL Error via CL','Total Error')
    axis tight
    
    % Open loop E field estimation comparison
    figure(55)
    plot(1:darkHole.pixelNum,real(data.EfocalEstOpenLoop(:,itr)),'r',...
        1:darkHole.pixelNum,real(EhatOL_10_viaE0Drift),'r--',...
        1:darkHole.pixelNum,real(EhatOL_10_viaCL),'b',...
        1:darkHole.pixelNum, real(EhatOL_10_viaProbedCL),'g');
    axis tight
    legend('$E^{OL}_{i}$','$\hat{E}^{OL}_0 + G\sum_{1}^{i} u^{drift}_i$',...
        '$\hat{E}^{CL}_{i} - G\sum_{1}^{i}(u^{EFC}_i + u^{dith}_i)$',...
        '$\hat{E}^{CL_p}_{i} - G\sum_{1}^{i}(u^{EFC}_i + u^{dith}_i)$','interpreter','latex','fontsize',14) 
    title('Step i Open Loop E Field and Estimation Methods');
    xlabel('Pixel')
    ylabel('Re(E field)')
    
 
    
    

end

