function [command, probeImage] = MultiSpeckleNulling(I, target, DM, coronagraph, camera, controller, darkHole, DM1command, DM2command, simOrLab)
% Probe one DM and take probing images
% Developed by He Sun on Sep. 30, 2018
%
% I - the measured image
% PSF- the point spread function core
% target - defines the properties of light source
% DM - defines the DM model and parameters of devices
% coronagraph - defines the coronagraph type, shape and distances
% camera - defines the properties of camera, including pixel size, binning,
% noises and others
% DM1command, DM2command - the current voltage commands of DMs
%%
centerEta = (camera.Neta + 1) / 2; % the center pixel coordinate
centerXi = (camera.Nxi + 1) / 2; % the center pixel coordinate
%%
if target.broadBandControl
    Iconv = zeros(size(I));
    for k = 1 : target.broadSampleNum
        Iw = I(:, :, k) .* darkHole.mask;
        PSF_normalization = sum(sum(controller.PSF(:, :, k).^2));
        Iconv(:, :, k) = conv2(Iw, controller.PSF(:, :, k), 'same') / PSF_normalization;
    end
    BW = imregionalmax(abs(Iconv), 26);
    ind = find(BW);
    [row, col, layer] = ind2sub(size(BW), ind);

    positionEta = zeros(length(row), 1);
    positionXi = zeros(length(row), 1);
    amp = zeros(length(row), 1);
    for ks = 1 : length(row)
        positionEta(ks) = (row(ks) - centerEta) * camera.binEta * camera.pitch / (target.starWavelengthBroad(layer(ks)) * coronagraph.focalLength / coronagraph.SPwidth); % vertical position of the brightest speckles in f * lamnda / D
        positionXi(ks) = (col(ks) - centerXi) * camera.binXi * camera.pitch / (target.starWavelengthBroad(layer(ks)) * coronagraph.focalLength / coronagraph.SPwidth); % horizontal position of the brightest speckles in f * lamnda / D
        amp(ks) = target.starWavelengthBroad(layer(ks)) / (2*pi) * sqrt(Iconv(row(ks), col(ks)));
    end
    visited = zeros(size(amp));
    subset = zeros(size(amp));
    for ks1 = 1 : length(row)
        if ~visited(ks1)
            visited(ks1) = 1;
            index = ks1;
            for ks2 = [1:ks1-1, ks1+1:length(row)]
                if sqrt((positionEta(ks1)-positionEta(ks2))^2 + (positionXi(ks1)-positionXi(ks2))^2) < 3 * camera.binEta * camera.pitch / (target.starWavelength * coronagraph.focalLength / coronagraph.SPwidth)
                    visited(ks2) = 1;
                    if amp(ks2) > amp(index)
                        index = ks2;
                    end
                end
            end
            subset(index) = 1;
        end
    end
    subsetIndices = find(subset);
    amp = amp(subsetIndices);
    positionEta = positionEta(subsetIndices);
    positionXi = positionXi(subsetIndices);
    row  = row(subsetIndices);
    col = col(subsetIndices);
    layer = layer(subsetIndices);
%     amp2 = amp;
%     for ks1 = 1 : length(row)
%         amp_normalization = 0;
%         for ks2 = 1 : length(row)
%             if sqrt((positionEta(ks1)-positionEta(ks2))^2 + (positionXi(ks1)-positionXi(ks2))^2) < 3 * camera.binEta * camera.pitch / (target.starWavelength * coronagraph.focalLength / coronagraph.SPwidth)
%                 amp_normalization = amp_normalization + amp(ks2);
%             end
%         end
%         amp2 = amp(ks1)^2/amp_normalization;
%     end
%     amp = amp2;
    % visualize matched filter result and speckle location
    figure(3), imagesc(mean(Iconv, 3)), colorbar;
    for ks = 1 : length(row)
        rectangle('Position', [col(ks)-2, row(ks)-2, 5, 5], 'Curvature', [1, 1], 'EdgeColor','r', 'LineStyle', '-.', 'LineWidth',1.5);
    end
    drawnow
else
    I = I .* darkHole.mask;
    PSF_normalization = sum(sum(controller.PSF.^2));
    Iconv = conv2(I, controller.PSF, 'same') / PSF_normalization;
    
    % detect the indices of multiple speckles
    BW = imregionalmax(abs(Iconv), 8);
    [argmax1, argmax2] = argmax(Iconv);
    BW = BW .* (Iconv > 0.4 * Iconv(argmax1, argmax2));
    [row, col] =  find(BW);
    positionEta = zeros(length(row), 1);
    positionXi = zeros(length(col), 1);
    amp = zeros(length(row), 1);
    for ks = 1 : length(row)
        positionEta(ks) = (row(ks) - centerEta) * camera.binEta * camera.pitch / (target.starWavelength * coronagraph.focalLength / coronagraph.SPwidth); % vertical position of the brightest speckles in f * lamnda / D
        positionXi(ks) = (col(ks) - centerXi) * camera.binXi * camera.pitch / (target.starWavelength * coronagraph.focalLength / coronagraph.SPwidth); % horizontal position of the brightest speckles in f * lamnda / D
        amp(ks) = target.starWavelength / (2*pi) * sqrt(Iconv(row(ks), col(ks)));
    end

    % visualize matched filter result and speckle location
    figure(3), imagesc(Iconv), colorbar;
    for ks = 1 : length(row)
        rectangle('Position', [col(ks)-2, row(ks)-2, 5, 5], 'Curvature', [1, 1], 'EdgeColor','r', 'LineStyle', '-.', 'LineWidth',1.5);
    end
    drawnow
end
%% compute the coordinate on the DM
dx = coronagraph.SPwidth / DM.DMmesh(2);
dy = coronagraph.SPwidth / DM.DMmesh(1);
xs = (-DM.DMmesh(2)/2 + 0.5 : DM.DMmesh(2)/2 - 0.5) * dx;
ys = (-DM.DMmesh(1)/2 + 0.5 : DM.DMmesh(1)/2 - 0.5) * dy;
[XS, YS] = meshgrid(xs, ys);

%% Find optimal offset by fitting the contrasts with a sine wave
Nitr = 5;
offset = [0, pi/2, pi, 3*pi/2];
contrasts = zeros(4, length(row));
if target.broadBandControl
    probeImage = zeros(camera.Neta, camera.Nxi, target.broadSampleNum, 4);
else
    probeImage = zeros(camera.Neta, camera.Nxi, 4);
end
for j = 1:4
    probeDM = controlShape(coronagraph, XS, YS, offset(j), amp, positionEta, positionXi);
    command = height2voltage(probeDM, DM.DMperfect, controller.whichDM, Nitr);
%     ItestBroad = zeros(camera.Neta, camera.Nxi, target.broadSampleNum);
%     for kWavelength = 1 : target.broadSampleNum
    if ~target.broadBandControl
        targetmon = target;
    %         targetmon.starWavelength = target.starWavelengthBroad(kWavelength);
        Itest = getImg(targetmon, DM, coronagraph, camera, DM1command + command, DM2command, simOrLab);
    %         ItestBroad(:, :, kWavelength) = Itest;
    %     end
    %     Itest = mean(Itest, 3);
        for ks = 1 : length(row)
            r = 2;
            contrasts(j, ks) =  mean(mean(Itest(row(ks)-r:row(ks)+r, col(ks)-r:col(ks)+r)));
        end
        probeImage(:, :, j) = Itest;
    else
        Itest = zeros(camera.Neta, camera.Nxi, target.broadSampleNum);
        targetmon = target;
        for kWavelength = 1 : target.broadSampleNum
            targetmon.starWavelength = target.starWavelengthBroad(kWavelength);
            Itest(:, :, kWavelength) = getImg(targetmon, DM, coronagraph, camera, DM1command + command, DM2command, simOrLab);
        end
        for ks = 1 : length(row)
            r = 2;
            contrasts(j, ks) =  mean(mean(Itest(row(ks)-r:row(ks)+r, col(ks)-r:col(ks)+r, layer(ks))));
        end
        probeImage(:, :, :, j) = Itest;
    end
end
% B = mean(contrasts);
% A = 0.5 * sqrt((contrasts(1)-contrast(3))^2 + (contrasts(2)-contrasts(4))^2);
theta = zeros(length(row), 1);
for ks = 1 : length(row)
    theta(ks) = atan2(contrasts(1, ks)-contrasts(3, ks), contrasts(2, ks)-contrasts(4, ks));
end
optimalOffset = mod((1.5*pi-theta), 2*pi);
probeDM = controlShape(coronagraph, XS, YS, optimalOffset, amp, positionEta, positionXi);
command = height2voltage(0.4*probeDM, DM, controller.whichDM, Nitr);
% Itest = getImg(target, DM, coronagraph, camera, DM1command + command, DM2command, simOrLab);
% mean(Itest(darkHole.pixelIndex))

%% Find optimal offset by line search
% Nitr = 5; % the iteration for computing DM command from probe surface shape
% numOffset = 36; % the number of offsets tested for the sine wave probing
% contrasts = zeros(numOffset, 1); % the measured contrast for different offsets
% for j = 1 : numOffset
%     offset = (j - 1) * (2 * pi / numOffset);
%     probeDM = controlShape(coronagraph, XS, YS, offset, amp, positionEta, positionXi);
%     command = height2voltage(probeDM, DM, controller.whichDM, Nitr);
%     Itest = getImg(target, DM, coronagraph, camera, DM1command + command, DM2command, simOrLab);
%     contrasts(j) =  mean(Itest(darkHole.pixelIndex));
% %     disp([j, contrasts(j)*1e4])
% end
% [~, opt] =  min(contrasts);
% optimalOffset = (opt - 1) * (2 * pi / numOffset);
% probeDM = controlShape(coronagraph, XS, YS, optimalOffset, amp, positionEta, positionXi);
% command = height2voltage(probeDM, DM, controller.whichDM, Nitr);
% Itest = getImg(target, DM, coronagraph, camera, DM1command + command, DM2command, simOrLab);
% mean(Itest(darkHole.pixelIndex))

end

function surf = controlShape(coronagraph, XS, YS, offset, amp, positionEta, positionXi)
% Calculate the sine wave control shape on the deformable mirror
% Developed by He Sun on Oct. 1, 2018
%
surf = zeros(size(XS));
for ks = 1 : length(amp)
    wy = 2 * pi / (coronagraph.SPwidth / positionEta(ks));
    wx = 2 * pi / (coronagraph.SPwidth / positionXi(ks));
    if length(offset)==1
        surf = surf + amp(ks) * sin(wx * XS + wy * YS + offset);
    else
        surf = surf + amp(ks) * sin(wx * XS + wy * YS + offset(ks));
    end
end
end


function command = height2voltage(surf, DM, index, Nitr)
% Use iterated calculation to find approximate DM command for a surface
% shape
%
% Select the DM gain matrix according to the DM index
switch index
    case '1'
        gain = DM.DM1gain;
    case '2'
        gain = DM.DM2gain;
    otherwise
        disp('The DM used for control should be either 1 or 2!');
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


function [argmax1, argmax2] = argmax(I)
% Find the max index of a 2-D array
% Developed by He Sun on Sep. 30, 2018
%
% I - the input 2-D array
[temp1, temp2] = max(I);
[~, argmax2] = max(temp1);
argmax1 = temp2(argmax2);
end