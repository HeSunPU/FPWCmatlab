function [imageAll, u, data] = takeProbingImagesBroad(contrastEst, target, DM, coronagraph, camera, darkHole, estimator, DM1command, DM2command, simOrLab, data)
% Probe one DM and take probing images
% Developed by He Sun on Feb. 22, 2017, revised from A. J. Riggs's
% "hcil_makeProbeSurf.m" file
%
% contrastEst - the estimated contrast, which is used to compute the
% amplitude of DM probing
% target - defines the properties of light source
% DM - defines the DM model and parameters of devices
% coronagraph - defines the coronagraph type, shape and distances
% camera - defines the properties of camera, including pixel size, binning,
% noises and others
% darkHole - defines the dark hole region
% estimator - defines the parameters of wavefront estimator
% DM1command, DM2command - the current voltage commands of DMs
%
%% initialize the output variables
if ((strcmpi(estimator.type, 'EKF') || strcmpi(estimator.type, 'UKF')) && estimator.EKFpairProbing == 0)
    imageAll = zeros(camera.Neta, camera.Nxi, 1 + estimator.NumImg, target.broadSampleNum); % unprobed and probed images
else
    imageAll = zeros(camera.Neta, camera.Nxi, 1 + 2 * estimator.NumImgPair, target.broadSampleNum); % unprobed and probed images
end
switch lower(estimator.type)
    case {'ekf', 'ukf'}
        u = zeros(DM.activeActNum, estimator.NumImg);
    otherwise
        u = zeros(DM.activeActNum, estimator.NumImgPair); % control commands for probing
end
Nitr = 5; % number of iterations used for calculating voltages of a specific shape

%% compute the coordinate on the DM
dx = coronagraph.SPwidth / DM.DMmesh(2);
dy = coronagraph.SPwidth / DM.DMmesh(1);
xs = (-DM.DMmesh(2)/2 + 0.5 : DM.DMmesh(2)/2 - 0.5) * dx;
ys = (-DM.DMmesh(1)/2 + 0.5 : DM.DMmesh(1)/2 - 0.5) * dy;
[XS, YS] = meshgrid(xs, ys);

%% take unprobed image
contrastMeasured = zeros(target.broadSampleNum, 1);
for kWavelength = 1 : target.broadSampleNum
    target.starWavelength = target.starWavelengthBroad(kWavelength);
    I = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab); % take the unprobed image
    % [~, ~, I] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command); % simulate the image
    imageAll(:, :, 1, kWavelength) = I; % save the unprobed image to the output variable
    contrastMeasured(kWavelength) = mean(I(darkHole.pixelIndex)); % the averaged measured contrast in the dark holes
end
disp('Unprobed image is taken.');
disp(['The averaged contrast in the dark holes is ', num2str(mean(contrastMeasured))]);

%% choose the probe contrast
if mean(contrastEst) <= 0 % if we currently do not have efficient estimation of contrast, for example the first iteration
    contrastEst = contrastMeasured;
end
contrastEst = contrastMeasured;

probeContrast = min(sqrt(mean(contrastEst) * 1e-5), 5e-4); % A heuristic law to calculate the probe contrast
disp(['Chosen probe contrast is ', num2str(probeContrast)]);
if data.itr > 0
    data.probeContrast(data.itr) = probeContrast;
end
%% compute the offsets of probed sinc waves
switch lower(estimator.probeMethod)
    case 'empirical'   
        switch lower(estimator.type)
            case {'ekf', 'ukf'}
                if (estimator.EKFpairProbing == 0)
                    omega = 1.5708/estimator.NumImg;
                    offsets = (omega * (data.itr-1) + (0 : estimator.NumImg-1))' * pi / estimator.NumImg;
                else
                    omega = 1.5708/estimator.NumImgPair;
                    offsets = (omega * (data.itr-1) + (0 : estimator.NumImgPair-1))' * pi / estimator.NumImgPair;
                end
            otherwise
%                 omega = 1.5708/estimator.NumImgPair;
                omega = 1.5708/2;
                offsets = (omega * (data.itr-1) + (0 : estimator.NumImgPair-1))' * pi / estimator.NumImgPair;
        end
    otherwise
        disp('The probing offset has not been assigned!');
        return;
end

%% take probed images
if ((strcmpi(estimator.type, 'EKF') || strcmpi(estimator.type, 'UKF')) && estimator.EKFpairProbing == 0)
    for k = 1 : estimator.NumImg
        target.starWavelength = mean(target.starWavelengthBroad);
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
        command = height2voltage(probeDM, DM.DMperfect, estimator.whichDM, Nitr);
        u(:, k) = command';
        % take images for positive or negative probing
        contrastProbe = zeros(target.broadSampleNum, 1);
        for kWavelength = 1 : target.broadSampleNum
            target.starWavelength = target.starWavelengthBroad(kWavelength);
            switch estimator.whichDM
                case '1'
                    Iprobe = getImg(target, DM, coronagraph, camera, DM1command + command, DM2command, simOrLab);
                case '2'
                    Iprobe = getImg(target, DM, coronagraph, camera, DM1command, DM2command + command, simOrLab);
                otherwise
                    disp('The DM used for probing should be either 1 or 2!');
                    return;
            end
            imageAll(:, :, k+1, kWavelength) = Iprobe;
            contrastProbe(kWavelength) = mean(Iprobe(darkHole.pixelIndex));
        end
        disp(['No. ', num2str(k), 'Probed image is taken.' ]);
        disp(['The averaged contrast in the dark holes of probed image is ', num2str(mean(contrastProbe))]);
    end
else
    %%
    for k = 1 : estimator.NumImgPair
        %%
        target.starWavelength = mean(target.starWavelengthBroad);
        probeSP = probeShape(target, coronagraph, estimator, XS, YS, offsets(k), probeContrast); % the desired probe shape on pupil plane in meters
        %%
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
        %%
        % compute the DM voltage command to generate such surface shape
        command = height2voltage(probeDM, DM.DMperfect, estimator.whichDM, Nitr);
        switch lower(estimator.type)
            case {'ekf', 'ukf'}
                u(:, 2*k - 1) = command';
                u(:, 2*k) = -command';
            otherwise
                u(:, k) = command';
        end
        %%
        % take images for positive or negative probing
        contrastPlus = zeros(target.broadSampleNum, 1);
        contrastMinus = zeros(target.broadSampleNum, 1);
               
        switch estimator.whichDM
            case '1'
                for kWavelength = 1 : target.broadSampleNum
                    target.starWavelength = target.starWavelengthBroad(kWavelength);
                    Iplus = getImg(target, DM, coronagraph, camera, DM1command + command, DM2command, simOrLab);
                    imageAll(:, :, 2 * k, kWavelength) = Iplus;
                    contrastPlus(kWavelength) = mean(Iplus(darkHole.pixelIndex));
                end
                for kWavelength = 1 : target.broadSampleNum
                    target.starWavelength = target.starWavelengthBroad(kWavelength);
                    Iminus = getImg(target, DM, coronagraph, camera, DM1command - command, DM2command, simOrLab);
                    imageAll(:, :, 2 * k + 1, kWavelength) = Iminus;
                    contrastMinus(kWavelength) = mean(Iminus(darkHole.pixelIndex));
                end
            case '2'
                for kWavelength = 1 : target.broadSampleNum
                    target.starWavelength = target.starWavelengthBroad(kWavelength);
                    Iplus = getImg(target, DM, coronagraph, camera, DM1command, DM2command + command, simOrLab);
                    imageAll(:, :, 2 * k, kWavelength) = Iplus;
                    contrastPlus(kWavelength) = mean(Iplus(darkHole.pixelIndex));
                end
                for kWavelength = 1 : target.broadSampleNum
                    target.starWavelength = target.starWavelengthBroad(kWavelength);
                    Iminus = getImg(target, DM, coronagraph, camera, DM1command, DM2command - command, simOrLab);
                    imageAll(:, :, 2 * k + 1, kWavelength) = Iminus;
                    contrastMinus(kWavelength) = mean(Iminus(darkHole.pixelIndex));
                end
            otherwise
                disp('The DM used for probing should be either 1 or 2!');
                return;
        end
        
        disp(['No. ', num2str(k), ' pair of probed images is taken.' ]);
        disp(['The averaged contrast in the dark holes of positive probed image is ', num2str(mean(contrastPlus))]);
        disp(['The averaged contrast in the dark holes of negative probed image is ', num2str(mean(contrastPlus))]);
    end
end
end


%% subfunctions used before

function surf = probeShape(target, coronagraph, estimator, XS, YS, offset, probeContrast)
% Calculate the probe shape
%
mx = (estimator.probeArea(2) - estimator.probeArea(1)) / coronagraph.SPwidth;
my = (estimator.probeArea(4) - estimator.probeArea(3)) / coronagraph.SPwidth; % frequency of the sinc wave, depending on the width of dark hole regions
wx = (estimator.probeArea(2) + estimator.probeArea(1)) / 2;
wy = (estimator.probeArea(4) + estimator.probeArea(3)) / 2; % frequency of the cosine wave, depending on the location of dark hole regions
SincAmp = target.starWavelength * sqrt(probeContrast) * sqrt(2 * pi); % amplitude of probe shape in meters;
if strcmpi(coronagraph.type, 'SPLC')
    surf = SincAmp * sinc(mx * (XS)) .* sinc(my * (YS)+2*pi) .* ... 
        cos(2 * pi * wx / coronagraph.SPwidth * XS + offset) .* cos(2 * pi * wy / coronagraph.SPwidth * YS);
else
    surf = SincAmp * sinc(mx * (XS)) .* sinc(my * (YS)) .* ... 
        cos(2 * pi * wx / coronagraph.SPwidth * XS + offset) .* cos(2 * pi * wy / coronagraph.SPwidth * YS);
end
surf = pi * surf; % A.J. added an extra factor in the old code, seems like some black art.
end

function command = height2voltage(surf, DM, index, Nitr)
% Use iterated calculation to find approximate DM command for a surface
% shape
% Select the DM gain matrix according to the DM index
switch index
    case '1'
        gain = DM.DM1gain;
    case '2'
        gain = DM.DM2gain;
    otherwise
        disp('The DM used for probing should be either 1 or 2!');
        return;
end
% use simple gradient optimization to calculate the voltage
command2D = imresize(surf, [DM.Nact, DM.Nact]) ./ gain;
command2D = command2D .* DM.DMstop;
for k = 1 : Nitr
    currentSurf = surfaceShape(DM, command2D, index);
    surfBias = surf - currentSurf;
    command2D = command2D + imresize(surfBias, [DM.Nact, DM.Nact]);
    command2D = command2D .* DM.DMstop;
end
command = command2D(DM.activeActIndex);
end
