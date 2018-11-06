function img = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab)
%% get image with specific DM command
% Developed by He Sun on Feb. 23, 2017
%
% target - defines the properties of light source
% DM - defines the DM model and parameters of devices
% coronagraph - defines the coronagraph type, shape and distances
% camera - defines the properties of camera, including pixel size, binning,
% noises and others
% darkHole - defines the dark hole region
% estimator - defines the parameters of wavefront estimator
% DM1command, DM2command - the current voltage commands of DMs
% simOrLab - 'simulation' for taking simulated image, 'lab' for take real
% lab image
%
%% check the DM commands don't exceed upper limit
assert(~any(isnan([DM1command; DM2command])), 'ERROR: DM COMMANDS EXCEED LIMIT!!');
assert(max(max(abs([DM1command; DM2command]))) <= DM.voltageLimit, 'ERROR: DM COMMANDS EXCEED LIMIT!!');

%% take simulated or lab image
switch lower(simOrLab)
    case 'simulation'
        img = getSimImg(target, DM, coronagraph, camera, DM1command, DM2command);
    case 'lab'
        img = getLabImg(target, DM, camera, DM1command, DM2command);
    otherwise
        disp('We currently only have two modes, simulation or lab.');
end
end

function I = getSimImg(target, DM, coronagraph, camera, DM1command, DM2command)
%% get simulated image with specific DM command
%
% add noises to the DM voltage input
if DM.noise == 1
    voltageNoise1 = DM.DMvoltageStd * DM1command .* randn(size(DM1command));
    voltageNoise2 = DM.DMvoltageStd * DM2command .* randn(size(DM1command));
    DM1command = DM1command + voltageNoise1;
    DM2command = DM2command + voltageNoise2;
end

% simulate the image
[~, ~, I] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
% I = imtranslate(I, 1*[rand(1), rand(1)]); % shift the images to simulate
% telescople jittering
if ~(strcmpi(coronagraph.type, 'SPLC'))
    I = I .* coronagraph.FPMmask;
end

% add noises to the image
if camera.noise == 1
    readoutNoise = camera.readoutstd * randn(size(I));
    Iphoton = I * target.flux * camera.exposure;
%     Iphoton = poissrnd(Iphoton) + readoutNoise;
    Iphoton = Iphoton + readoutNoise;
%     I = float(Iphoton) / (target.flux * camera.exposure);
%     I = I + readoutNoise/(target.flux * camera.exposure);
%     Iphoton = ceil(Iphoton);
    I = Iphoton / (target.flux * camera.exposure);
end
end

function I = getLabImg(target, DM, camera, DM1command, DM2command)
%% get lab image with specific DM command
%
%% Send commands to deformable mirrors
% flip or rotate the DM commands
DM1command2D = zeros(DM.Nact, DM.Nact);
DM2command2D = zeros(DM.Nact, DM.Nact);
DM1command2D(DM.activeActIndex) = DM1command;
DM2command2D(DM.activeActIndex) = DM2command;
DM1command2D = fliplr(DM1command2D);
DM2command2D = rot90(DM2command2D, 2);
DM1command = DM1command2D(DM.activeActIndex);
DM2command = DM2command2D(DM.activeActIndex);
% send commands to DMs
DM1Voltage = DM.DM1bias + DM1command;
DM2Voltage = DM.DM2bias + DM2command; % calculate the true voltage inputs by adding command to flat voltage
voltages = [DM1Voltage; zeros(1024-952, 1); DM2Voltage; zeros(1024-952, 1); zeros(2048, 1)]; % vectorized input to DM driver
BitsCommand = ((2^16-1)/300) * voltages; % convert the command in voltages to bits
BrdNum = 1;
error = BurstHVA4096Frame1D(BrdNum, BitsCommand);

%% take lab image using QSI camera
I = takeImg(camera.handle, camera.stacking, camera.exposure, camera.startPosition, camera.imageSize, [camera.binXi, camera.binEta]);
if (max(max(I))> 3.3e5) % check whether the camera is saturated
    disp('The camera image is saturated!! STOP!!');
    return;
end
I = rot90(I - camera.darkFrame, 1); % subtract the dark frame
if mod(camera.Nxi,2) == 0
    xiCrop = [-camera.Nxi/2 + 1, camera.Nxi/2];
else xiCrop = [-floor(camera.Nxi/2), floor(camera.Nxi/2)];
end
if mod(camera.Neta,2) == 0
    etaCrop = [-camera.Neta/2 + 1, camera.Neta/2];
else etaCrop = [-floor(camera.Neta/2), floor(camera.Neta/2)];
end
I = I(camera.center(1) + etaCrop(1) : camera.center(1) + etaCrop(2), ...
    camera.center(2) + xiCrop(1) : camera.center(2) + xiCrop(2)); % crop the camera output to specific size
I = rot90(I, 2); % adjust the orientation of the image
I = double(I) / (target.flux * camera.exposure);

end