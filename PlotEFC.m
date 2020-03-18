%% Visualizations used for running experiment
% Author: Susan Redmond 
% Date: March 17, 2019 [Day 2 of isolation]
% Description:
% Plots visuals for runEFC.  If estimator.saveData = 1, the initial, middle, 
% and final figures are saved to the folder.dataLibrary path.
% note: setting the figure position may need to change depending on the
% computer running the code

%%
curFig = figure(1); 
set( curFig, 'Position', [15 100 900 500]);
if target.broadBandControl
    figure(1), imagesc(log10(abs(mean(data.I(:, :, :, itr), 3)))), colorbar
else
    figure(1), subplot(2,3,1), imagesc(log10(abs(data.I(:,:,itr)))), colorbar;
end
caxis(cRange);
title(['After control iteration ', num2str(itr)]);

subplot(2,3,2), imagesc(log10(abs(data.dImeasured2D(:,:,itr)))), colorbar;
title('Measured Intensity Change');
caxis(cRange);
subplot(2,3,5), imagesc(log10(abs(data.dImodel2D(:,:,itr)))), colorbar;
title('Model-predicted Intensity Change');
caxis(cRange);
    
subplot(2,3,3), imagesc(log10(abs(data.IincoEst2D(:,:,itr)))), colorbar;
caxis(cRange);
title(['Incoherent light after control iteration ', num2str(itr)]);

subplot(2,3,6), imagesc(log10(abs(data.IcoEst2D(:,:,itr)))), colorbar;
caxis(cRange);
title(['Coherent light after control iteration ', num2str(itr)]);


if target.broadBandControl
    %     figure(2), semilogy(0:itr, mean([data.contrast0, data.measuredContrastAverage(:, 1:itr)], 1), '-o' ,0:itr, mean([data.estimatedContrastAverage0, data.estimatedContrastAverage(:, 1:itr)], 1), '-s', 0:itr, mean([data.estimatedIncoherentAverage0, data.estimatedIncoherentAverage(:, 1:itr)], 1), '-^');
else
    
    subplot(2,3,4), semilogy(0:itr, [data.contrast0; data.measuredContrastAverage(1:itr)],'-o',...
        0:itr, [data.estimatedContrastAverage0; data.estimatedContrastAverage(1:itr)],'-s', ...
        0:itr, [data.estimatedIncoherentAverage0; data.estimatedIncoherentAverage(1:itr)], '-^');
end
ylim([10^(cRange(1)), 10^(cRange(2))]);
legend('measured', 'estimated', 'incoherent');
xlabel('iteration','interpreter','latex');
ylabel('contrast $[I/I_{max}]$','interpreter','latex');
drawnow
% curFig = gcf;


%% Save
if  any([1,floor(Nitr/2),Nitr] - itr == 0) && estimator.saveData == 1
    if itr == 1
        startTime = datestr(now,'_HH_MM');
    end
    figName = [folder.dataLibrary,'\dataSnapshotItr',num2str(itr),'Run',...
        num2str(runTrial),startTime];
    saveas(curFig,[figName,'.fig']);
    saveas(curFig,[figName,'.jpg']);
end

















% datestr(now,'HH_MM')