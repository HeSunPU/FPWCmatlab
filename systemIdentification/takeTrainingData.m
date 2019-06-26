%% Used to collect focal plane images for system identification
% Developed by He Sun on Mar. 12, 2017
%

clear data

%% Take first image and check current contrast
simOrLab = 'lab';
% coronagraph.FPMmask = createMask(target, coronagraph, camera, 'wedge', ...
%     'LR', [0, 0], [0, 0], [5, 11], 45); % generate the 2D focal plane mask
% DM1command = zeros(DM.activeActNum, 1);
% DM2command = zeros(DM.activeActNum, 1);
I0 = getImg(target, DM, coronagraph, camera, DM1command0, DM2command0, simOrLab);
contrast0 = mean(I0(darkHole.pixelIndex));
%% Choose the probing shape according to the contrast
numImgPair = 2;
probeContrast = min(sqrt(contrast0 * 1e-5), 5e-4); % A heuristic law to calculate the probe contrast
disp(['Chosen probe contrast is ', num2str(probeContrast)]);
uProbe = zeros(DM.activeActNum, numImgPair);
% compute the coordinate on the DM
dx = coronagraph.SPwidth / DM.DMmesh(2);
dy = coronagraph.SPwidth / DM.DMmesh(1);
xs = (-DM.DMmesh(2)/2 + 0.5 : DM.DMmesh(2)/2 - 0.5) * dx;
ys = (-DM.DMmesh(1)/2 + 0.5 : DM.DMmesh(1)/2 - 0.5) * dy;
[XS, YS] = meshgrid(xs, ys);
% compute the offset of the probed sinc waves
offsets = (0 : numImgPair-1)' * pi / numImgPair;
for k = 1 : numImgPair
    probeSP = probeShape(target, coronagraph, estimator, XS, YS, offsets(k), probeContrast); % the desired probe shape on pupil plane in meters
    % Since the width of shaped pupil is different from DM, we should
    % adjust the probe shape to DM plane
    if coronagraph.SPwidth >= DM.widthDM
        marginWidth = (coronagraph.SPwidth - DM.widthDM)/2;
        marginNpixel = round(marginWidth / coronagraph.SPwidth * DM.DMmesh(1));
        probeSPcrop = probeSP(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel);
        probeDM = imresize(probeSPcrop, DM.DMmesh);
    else
        marginWidth = (DM.widthDM - coronagraph.SPwidth)/2;
        marginNpixel = round(marginWidth / DM.widthDM * DM.DMmesh(1));
        probeSPresized = imresize(probeSP, DM.DMmesh - 2 * marginNpixel);
        probeDM = zeros(DM.DMmesh);
        probeDM(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = probeSPresized;
    end
    % compute the DM voltage command to generate such surface shape
    command = height2voltage(probeDM, DM, estimator.whichDM, 5);
    uProbe(:, k) = command';
end
%% Initialize the data structure to save commands and images
nStep = 4000;
% nStep = length(DM1command);
data.u1 = zeros(DM.activeActNum, nStep);
data.u2 = zeros(DM.activeActNum, nStep);
data.image = zeros(darkHole.pixelNum, numImgPair, nStep);
data.imageNoProbing = zeros(darkHole.pixelNum, nStep);
% data.Efocal = zeros(darkHole.pixelNum, nStep);
data.image0 = zeros(darkHole.pixelNum, numImgPair);
data.I0 = I0(darkHole.pixelIndex);
data.Iplus0 = zeros(darkHole.pixelNum, numImgPair);
data.Iplus = zeros(darkHole.pixelNum, numImgPair, nStep);
data.Iminus0 = zeros(darkHole.pixelNum, numImgPair);
data.Iminus = zeros(darkHole.pixelNum, numImgPair, nStep);
data.uProbe0 = uProbe;
data.uProbe = zeros(DM.activeActNum, numImgPair, nStep);%uProbe;
if strcmpi(simOrLab, 'simulation')
    data.Efocal = zeros(darkHole.pixelNum, nStep);
end
%%
DM1command = DM1command0;
DM2command = DM2command0;
% take images to estimate original contrast
for itrProbe = 1 : numImgPair
    Iplus = getImg(target, DM, coronagraph, camera, DM1command + uProbe(:, itrProbe), DM2command, simOrLab);
    Iminus = getImg(target, DM, coronagraph, camera, DM1command - uProbe(:, itrProbe), DM2command, simOrLab);
%     Iplus = getImg(target, DM, coronagraph, camera, DM1command, DM2command + uProbe(:, itrProbe), simOrLab);
%     Iminus = getImg(target, DM, coronagraph, camera, DM1command, DM2command - uProbe(:, itrProbe), simOrLab);
    Idiff = Iplus - Iminus;
    data.image0(:, itrProbe) = Idiff(darkHole.pixelIndex);
    data.Iplus0(:, itrProbe) = Iplus(darkHole.pixelIndex);
    data.Iminus0(:, itrProbe) = Iminus(darkHole.pixelIndex);
end
% [EfocalStar, EfocalPlanet, I0] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
% Efocal0 = EfocalStar(darkHole.pixelIndex);
%%
% take data step by step
for itr = 1 : nStep
    %%
    % compute the offset of the probed sinc waves
    offsets = 2*pi*rand() + ((0 : numImgPair-1)' * pi / numImgPair);
    for k = 1 : numImgPair
        probeSP = probeShape(target, coronagraph, estimator, XS, YS, offsets(k), probeContrast); % the desired probe shape on pupil plane in meters
        % Since the width of shaped pupil is different from DM, we should
        % adjust the probe shape to DM plane
        if coronagraph.SPwidth >= DM.widthDM
            marginWidth = (coronagraph.SPwidth - DM.widthDM)/2;
            marginNpixel = round(marginWidth / coronagraph.SPwidth * DM.DMmesh(1));
            probeSPcrop = probeSP(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel);
            probeDM = imresize(probeSPcrop, DM.DMmesh);
        else
            marginWidth = (DM.widthDM - coronagraph.SPwidth)/2;
            marginNpixel = round(marginWidth / DM.widthDM * DM.DMmesh(1));
            probeSPresized = imresize(probeSP, DM.DMmesh - 2 * marginNpixel);
            probeDM = zeros(DM.DMmesh);
            probeDM(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = probeSPresized;
        end
        % compute the DM voltage command to generate such surface shape
        command = height2voltage(probeDM, DM, estimator.whichDM, 5);
        uProbe(:, k) = command';
    end
    data.uProbe(:, :, itr) = uProbe;
    %%
    singlePoke = zeros(size(DM1command0));
    singlePoke(itr) = 1;
%     DM1command = DM1command0 + 0.3 * singlePoke;
    DM1command = DM1command0 + 0.3 * (rand(size(DM1command))-0.5);
    DM2command = DM2command0 + 0.3 * (rand(size(DM2command))-0.5);
    if itr == 1
        data.u1(:, itr) = DM1command - DM1command0;
        data.u2(:, itr) = DM2command - DM2command0;
    else
        data.u1(:, itr) = DM1command - DM1commandOld;
        data.u2(:, itr) = DM2command - DM2commandOld;
    end
    DM1commandOld = DM1command;
    DM2commandOld = DM2command;
    if strcmpi(simOrLab, 'simulation')
        [EfocalStar, EfocalPlanet, I0] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
        data.Efocal(:, itr) = EfocalStar(darkHole.pixelIndex);
%         figure(1), imagesc(log10(abs(I0))), colorbar;
%         drawnow
    end
    I0 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
    figure(1), imagesc(log10(abs(I0))), colorbar
    drawnow
    data.imageNoProbing(:, itr) = I0(darkHole.pixelIndex);
    contrast = mean(I0(darkHole.pixelIndex));
    if (contrast > 6e-5)
        return;
    end
    disp(['The average contrast after Iteration ', num2str(itr), ' is ', num2str(contrast)]);
    for itrProbe = 1 : numImgPair
%         if mod(itr, 2) == 0
            Iplus = getImg(target, DM, coronagraph, camera, DM1command + uProbe(:, itrProbe), DM2command, simOrLab);
            Iminus = getImg(target, DM, coronagraph, camera, DM1command - uProbe(:, itrProbe), DM2command, simOrLab);
%         else
%             Iplus = getImg(target, DM, coronagraph, camera, DM1command, DM2command + uProbe(:, itrProbe), simOrLab);
%             Iminus = getImg(target, DM, coronagraph, camera, DM1command, DM2command - uProbe(:, itrProbe), simOrLab);
%         end
%         Iplus = getImg(target, DM, coronagraph, camera, DM1command, DM2command + uProbe(:, itrProbe), simOrLab);
%         figure(2), imagesc(log10(abs(Iplus))), colorbar;
%         drawnow
        
%         Iminus = getImg(target, DM, coronagraph, camera, DM1command, DM2command - uProbe(:, itrProbe), simOrLab);
%         figure(2), imagesc(log10(abs(Iplus))), colorbar
%         drawnow
        Idiff = Iplus - Iminus;
        data.image(:, itrProbe, itr) = Idiff(darkHole.pixelIndex);
        data.Iplus(:, itrProbe, itr) = Iplus(darkHole.pixelIndex);
        data.Iminus(:, itrProbe, itr) = Iminus(darkHole.pixelIndex);
    end
end
%%
% dataDM1dark = data;
% save dataDM1dark dataDM1dark



% %% Used to collect focal plane images for system identification
% % Developed by He Sun on Mar. 12, 2017
% %
% 
% clear data
% 
% %% Take first image and check current contrast
% simOrLab = 'lab';
% % coronagraph.FPMmask = createMask(target, coronagraph, camera, 'wedge', ...
% %     'LR', [0, 0], [0, 0], [5, 11], 45); % generate the 2D focal plane mask
% % DM1command = zeros(DM.activeActNum, 1);
% % DM2command = zeros(DM.activeActNum, 1);
% I0 = getImg(target, DM, coronagraph, camera, DM1command0, DM2command0, simOrLab);
% contrast0 = mean(I0(darkHole.pixelIndex));
% %% Choose the probing shape according to the contrast
% numImgPair = 2;
% probeContrast = min(sqrt(contrast0 * 1e-5), 5e-4); % A heuristic law to calculate the probe contrast
% disp(['Chosen probe contrast is ', num2str(probeContrast)]);
% uProbe = zeros(DM.activeActNum, numImgPair);
% % compute the coordinate on the DM
% dx = coronagraph.SPwidth / DM.DMmesh(2);
% dy = coronagraph.SPwidth / DM.DMmesh(1);
% xs = (-DM.DMmesh(2)/2 + 0.5 : DM.DMmesh(2)/2 - 0.5) * dx;
% ys = (-DM.DMmesh(1)/2 + 0.5 : DM.DMmesh(1)/2 - 0.5) * dy;
% [XS, YS] = meshgrid(xs, ys);
% % compute the offset of the probed sinc waves
% offsets = (0 : numImgPair-1)' * pi / numImgPair;
% for k = 1 : numImgPair
%     probeSP = probeShape(target, coronagraph, estimator, XS, YS, offsets(k), probeContrast); % the desired probe shape on pupil plane in meters
%     % Since the width of shaped pupil is different from DM, we should
%     % adjust the probe shape to DM plane
%     if coronagraph.SPwidth >= DM.widthDM
%         marginWidth = (coronagraph.SPwidth - DM.widthDM)/2;
%         marginNpixel = round(marginWidth / coronagraph.SPwidth * DM.DMmesh(1));
%         probeSPcrop = probeSP(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel);
%         probeDM = imresize(probeSPcrop, DM.DMmesh);
%     else
%         marginWidth = (DM.widthDM - coronagraph.SPwidth)/2;
%         marginNpixel = round(marginWidth / DM.widthDM * DM.DMmesh(1));
%         probeSPresized = imresize(probeSP, DM.DMmesh - 2 * marginNpixel);
%         probeDM = zeros(DM.DMmesh);
%         probeDM(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = probeSPresized;
%     end
%     % compute the DM voltage command to generate such surface shape
%     command = height2voltage(probeDM, DM, estimator.whichDM, 5);
%     uProbe(:, k) = command';
% end
% %% Initialize the data structure to save commands and images
% % nStep = 4000;
% nStep = length(DM1command);
% data.u1 = zeros(DM.activeActNum, nStep);
% data.u2 = zeros(DM.activeActNum, nStep);
% data.image = zeros(darkHole.pixelNum, numImgPair, nStep);
% data.imageNoProbing = zeros(darkHole.pixelNum, nStep);
% % data.Efocal = zeros(darkHole.pixelNum, nStep);
% data.image0 = zeros(darkHole.pixelNum, numImgPair);
% data.I0 = I0(darkHole.pixelIndex);
% data.Iplus0 = zeros(darkHole.pixelNum, numImgPair);
% data.Iplus = zeros(darkHole.pixelNum, numImgPair, nStep);
% data.Iminus0 = zeros(darkHole.pixelNum, numImgPair);
% data.Iminus = zeros(darkHole.pixelNum, numImgPair, nStep);
% data.uProbe = uProbe;
% if strcmpi(simOrLab, 'simulation')
%     data.Efocal = zeros(darkHole.pixelNum, nStep);
% end
% %%
% DM1command = DM1command0;
% DM2command = DM2command0;
% % take images to estimate original contrast
% for itrProbe = 1 : numImgPair
% %     Iplus = getImg(target, DM, coronagraph, camera, DM1command + uProbe(:, itrProbe), DM2command, simOrLab);
% %     Iminus = getImg(target, DM, coronagraph, camera, DM1command - uProbe(:, itrProbe), DM2command, simOrLab);
%     Iplus = getImg(target, DM, coronagraph, camera, DM1command, DM2command + uProbe(:, itrProbe), simOrLab);
%     Iminus = getImg(target, DM, coronagraph, camera, DM1command, DM2command - uProbe(:, itrProbe), simOrLab);
%     Idiff = Iplus - Iminus;
%     data.image0(:, itrProbe) = Idiff(darkHole.pixelIndex);
%     data.Iplus0(:, itrProbe) = Iplus(darkHole.pixelIndex);
%     data.Iminus0(:, itrProbe) = Iminus(darkHole.pixelIndex);
% end
% % [EfocalStar, EfocalPlanet, I0] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
% % Efocal0 = EfocalStar(darkHole.pixelIndex);
% %%
% % take data step by step
% for itr = 1 : nStep
%     singlePoke = zeros(size(DM1command0));
%     singlePoke(itr) = 1;
%     DM1command = DM1command0;% + 0.3 * singlePoke;
% %     DM1command = DM1command0 + 0.3 * rand(size(DM1command));
%     DM2command = DM2command0 + 0.3 * singlePoke;
% %     DM2command = DM2command0 + 0.3 * rand(size(DM2command));
%     if itr == 1
%         data.u1(:, itr) = DM1command - DM1command0;
%         data.u2(:, itr) = DM2command - DM2command0;
%     else
%         data.u1(:, itr) = DM1command - DM1commandOld;
%         data.u2(:, itr) = DM2command - DM2commandOld;
%     end
%     DM1commandOld = DM1command;
%     DM2commandOld = DM2command;
%     if strcmpi(simOrLab, 'simulation')
%         [EfocalStar, EfocalPlanet, I0] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
%         data.Efocal(:, itr) = EfocalStar(darkHole.pixelIndex);
% %         figure(1), imagesc(log10(abs(I0))), colorbar;
% %         drawnow
%     end
%     I0 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
%     figure(1), imagesc(log10(abs(I0))), colorbar
%     drawnow
%     data.imageNoProbing(:, itr) = I0(darkHole.pixelIndex);
%     contrast = mean(I0(darkHole.pixelIndex));
%     if (contrast > 1.5e-5)
%         return;
%     end
%     disp(['The average contrast after Iteration ', num2str(itr), ' is ', num2str(contrast)]);
%     for itrProbe = 1 : numImgPair
% %         Iplus = getImg(target, DM, coronagraph, camera, DM1command + uProbe(:, itrProbe), DM2command, simOrLab);
%         Iplus = getImg(target, DM, coronagraph, camera, DM1command, DM2command + uProbe(:, itrProbe), simOrLab);
% %         figure(2), imagesc(log10(abs(Iplus))), colorbar;
% %         drawnow
% %         Iminus = getImg(target, DM, coronagraph, camera, DM1command - uProbe(:, itrProbe), DM2command, simOrLab);
%         Iminus = getImg(target, DM, coronagraph, camera, DM1command, DM2command - uProbe(:, itrProbe), simOrLab);
% %         figure(2), imagesc(log10(abs(Iplus))), colorbar
% %         drawnow
%         Idiff = Iplus - Iminus;
%         data.image(:, itrProbe, itr) = Idiff(darkHole.pixelIndex);
%         data.Iplus(:, itrProbe, itr) = Iplus(darkHole.pixelIndex);
%         data.Iminus(:, itrProbe, itr) = Iminus(darkHole.pixelIndex);
%     end
% end
% 
% %%
% dataDM2BothSinglePoke = data;
% save dataDM2BothSinglePoke dataDM2BothSinglePoke
% 
