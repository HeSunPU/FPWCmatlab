%% Initialization.m
% Cleaned on Oct. 31, 2018
% Developed by He Sun on Feb. 7th, 2017
%
% Initialize the setup of the simulation or the experiment, including the optical model, DM
% model and so on;
%

%% Initialize the path, should change for different computers
computerID = 'ultron'; % 'ultron', 'hesun', or 'hesunLaptop'
switch lower(computerID)
    case 'hesun'
        folder.main = 'C:\Users\hesun\Google Drive\Kasdin_lab\FPWC';
        folder.optics = 'C:\Users\hesun\Google Drive\Kasdin_lab\FPWC\opticalModel';
        folder.DM = 'C:\Users\hesun\Google Drive\Kasdin_lab\FPWC\DMmodel';
        folder.SSM = 'C:\Users\hesun\Google Drive\Kasdin_lab\FPWC\stateSpaceModel';
        folder.controller = 'C:\Users\hesun\Google Drive\Kasdin_lab\FPWC\controller';
        folder.estimator = 'C:\Users\hesun\Google Drive\Kasdin_lab\FPWC\estimator';
        folder.hardware = 'C:\Users\hesun\Google Drive\Kasdin_lab\FPWC\hardware';
        folder.dataLibrary = 'C:\Users\hesun\Google Drive\Kasdin_lab\FPWC\dataLibrary';
        folder.LOWFS = 'C:\Users\hesun\Google Drive\Kasdin_lab\FPWC\LOWFS';
    case 'ultron'
        folder.main = 'C:\Lab\FPWCmatlab';
        folder.optics = 'C:\Lab\FPWCmatlab\opticalModel';
        folder.DM = 'C:\Lab\FPWCmatlab\DMmodel';
        folder.SSM = 'C:\Lab\FPWCmatlab\stateSpaceModel';
        folder.controller = 'C:\Lab\FPWCmatlab\controller';
        folder.estimator = 'C:\Lab\FPWCmatlab\estimator';
        folder.hardware = 'C:\Lab\FPWCmatlab\hardware';
        folder.dataLibrary = 'C:\Lab\FPWCmatlab\dataLibrary\20181101';
        folder.LOWFS = 'C:\Lab\FPWCmatlab\LOWFS';
    case 'hesunlaptop'
        folder.main = pwd;
        folder.optics = [pwd, '\opticalModel'];
        folder.DM = [pwd, '\DMmodel'];
        folder.SSM = [pwd, '\stateSpaceModel'];
        folder.controller = [pwd, '\controller'];
        folder.estimator = [pwd, '\estimator'];
        folder.hardware = [pwd, '\hardware'];
        folder.dataLibrary = [pwd, '\dataLibrary\09122018'];
        folder.LOWFS = [pwd, '\LOWFS'];
    otherwise
        disp('The computer ID you give is not correct!!');
        return;
end
addpath(folder.optics);
addpath(folder.DM);
addpath(folder.dataLibrary);
addpath(folder.SSM);
addpath(folder.controller);
addpath(folder.estimator);
addpath(folder.hardware);
% addpath(folder.LOWFS);

%% The parameters for the optical model
% including light source, DM model, coronagraph shape and distances

%% Initialize the parameter for the DM
DM.model = 'influencingFunction'; % 'influencingFuntion' (or 'FEM', 'neuralNet')
DM.DMmesh = [442, 442]; % The pixel number in each direction should better be even, [442, 442] is a good choice to recover surface shape using linear superposition
DM.Nact = 34; % number of actuators in one direction
DM.pitch = 301e-6;%282.7e-6;%271e-6;%276.1e-6;%273.2e-6;%281.3e-6; % pitch size of DM actuator in meters
DM.widthDM = DM.pitch * DM.Nact; % DM width in meters, usually the DM is square
DM.DM1gain = 5.06e-9 * (ones(DM.Nact, DM.Nact)+ 0.6*(rand(DM.Nact, DM.Nact)-0.5));%5.06e-9 * (ones(DM.Nact, DM.Nact) + 0.8 * (rand(DM.Nact, DM.Nact)-0.5));% %6.27e-9 * ones(DM.Nact, DM.Nact);
DM.DM2gain = 6.27e-9 * (ones(DM.Nact, DM.Nact)+ 0.6*(rand(DM.Nact, DM.Nact)-0.5)); % the DM gain (voltage to height) of each actuator, unit: meter / volt
% temp = load('DM1gain.mat');
% DM.DM1gain = temp.DM1gain;
% temp = load('DM2gain.mat');
% DM.DM2gain = temp.DM2gain;

DM.zDM1toDM2 = 0.23;%0.33697;%0.42545;%simulation shows 0.22 may be the best % distance from  DM1 to DM2 in meters
DM.voltageLimit = 50; % the limt for DM voltage in volts
DM.DMstop = zeros(DM.Nact, DM.Nact); % The DM actuators are located in a circular region
for m = 1 : DM.Nact
    for n = 1 : DM.Nact
        if(sqrt((m - 17.5)^2 + (n - 17.5)^2) < 17.5)
            DM.DMstop(m, n) = 1;
        end
    end
end
DM.activeActIndex = find(DM.DMstop(:) == 1); % indices of actuators located in the circular are
DM.activeActNum = length(DM.activeActIndex); % number of actuators located in the circular area
DM.DM1command = zeros(DM.activeActNum, 1); 
DM.DM2command = zeros(DM.activeActNum, 1);
DM.DMvoltageStd = 0.01; % DM voltage std as the ratio of control command
DM.noise = 0; % 1 stands for that we add virtual noises during simulation
DM.DMperfect = DM;
% DM.DMperfect.DM1gain = 5.06e-9 * ones(DM.Nact, DM.Nact);%5.06e-9 * (ones(DM.Nact, DM.Nact) + 0.8 * (rand(DM.Nact, DM.Nact)-0.5));% %6.27e-9 * ones(DM.Nact, DM.Nact);
% DM.DMperfect.DM2gain = 6.27e-9 * ones(DM.Nact, DM.Nact); % the DM gain (voltage to height) of each actuator, unit: meter / volt

%% Initialize the coronagraph instrument layout
coronagraph.type = 'SPLC';%'SPC';%

if strcmpi(coronagraph.type, 'SPC') % 'Shaped pupil coroangraph'
    coronagraph.SPwidth = 0.01; % width of shaped pupil mask in meters
    coronagraph.Nsp = round(coronagraph.SPwidth / DM.widthDM * DM.DMmesh(1)); % number of pixels in one direction of shaped pupil mask matrix
    coronagraph.zDM2toSP = 0;%0.5334;%0.51308;%0.581; % distance from DM2 to shaped pupil mask in meters
    coronagraph.focalLength = 1.1642;%1.13;%1.1528;%1.141;%1.1379; %1.1652;%1.85; % focal length in meters
    coronagraph.SPshape = load([folder.optics '/SPs/ripple3_256x256_ideal_undersized.txt']);
    % coronagraph.SPshape = MakeMaskEllipse12b(coronagraph.Nsp/2, folder);
    % coronagraph.FPM = ones(camera.Nxi, camera.Neta);
end
if strcmpi(coronagraph.type, 'SPLC') % 'Shaped pupil Lyot coronagraph'
    coronagraph.SPwidth = 0.01; % width of shaped pupil mask in meters
    coronagraph.Nsp = round(coronagraph.SPwidth / DM.widthDM * DM.DMmesh(1)); % number of pixels in one direction of shaped pupil mask matrix
    coronagraph.zDM2toSP = 0;%0.5334;%0.51308;%0.581; % distance from DM2 to shaped pupil mask in meters
    coronagraph.focalLength = 1.1642;%1.13;%1.1528;%1.141;%1.1379; %1.1652;%1.85; % focal length in meters
    coronagraph.apertureWidth = coronagraph.SPwidth;
    coronagraph.Naperture = coronagraph.Nsp;
    coronagraph.Nfpm = 160;
    coronagraph.FPMpitch = 8.3168e-06;
    coronagraph.FPwidth = coronagraph.Nfpm * coronagraph.FPMpitch;
    coronagraph.lyotWidth = coronagraph.SPwidth;
    coronagraph.Nlyot = coronagraph.Nsp;
    coronagraph.apertureMask = fitsread('TestPupil.fits');
    coronagraph.SPshape = fitsread('TestSP.fits');
    coronagraph.FPmask = fitsread('TestFPM_990by990.fits');
    coronagraph.LyotStop = fitsread('TestLS_1000by1000.fits');
    coronagraph.apertureMask = imresize(coronagraph.apertureMask, [coronagraph.Nsp, coronagraph.Nsp], 'bicubic');
    coronagraph.SPshape = imresize(coronagraph.SPshape, [coronagraph.Nsp, coronagraph.Nsp], 'bicubic');
    coronagraph.LyotStop = imresize(coronagraph.LyotStop, coronagraph.Nlyot*[1, 1], 'bicubic');
    coronagraph.FPmask = imresize(coronagraph.FPmask, coronagraph.Nfpm*[1, 1], 'bicubic');
end

% add aberrations to the coronagraph
coronagraph.error = 1; % 1 stand for using error map
% coronagraph.aberration = ones(coronagraph.Nsp, coronagraph.Nsp);
if (coronagraph.error)
    % wavefront perturbation on DM1, DM2 and shaped pupil plane
    addpath([folder.optics, '/errorMap']);
    load PSD_DM1.mat;
    load PSD_DM2.mat;
    load PSD_SP.mat;
    coronagraph.DM1error = 1*PSD_DM1;
    coronagraph.DM2error = 1*PSD_DM2;
    coronagraph.SPerror = 1*PSD_SP;
%     coronagraph.SPerror = PSD_SP + 0.2 * mean(mean(abs(PSD_SP))) * rand(size(PSD_SP));
    
%     SPMaskDefect = coronagraph.SPshape;
%     SPindex = find(abs(coronagraph.SPshape-0.5) <0.49);
%     SPMaskDefect(SPindex) = min(SPMaskDefect(SPindex)+0.3, 1.1);
%     coronagraph.SPshape = SPMaskDefect;

end
coronagraph.coronagraph_perfect = coronagraph;
coronagraph.coronagraph_perfect.error = 0;
%% Initialize the parameters for camera
camera.pitch = 4.54e-6; % pitch size of the camera pixel in meters
camera.binXi = 4; % pixel binning in x direction
camera.binEta = 4; % pixel binning in y direction
camera.imageSize = [500, 500]; % the total image size in pixels get from camera
camera.startPosition = [0, 0]; % the starting camera pixel in taking images
camera.visionXi = [-12, 12]; % field of vision in horizontal direction, defined by how many f*lambda/D
camera.visionEta = [-10, 10]; % field of vision in vertical direction, defined by how many f*lambda/D
% camera.Nxi = ceil((camera.visionXi(2) - camera.visionXi(1)) * target.starWavelength * coronagraph.focalLength ...
%     / coronagraph.SPwidth / (camera.pitch * camera.binXi)); % number of pixels in horizontal direction in focal plane
% camera.Neta = ceil((camera.visionEta(2) - camera.visionEta(1)) * target.starWavelength * coronagraph.focalLength ...
%     / coronagraph.SPwidth / (camera.pitch * camera.binEta)); % number of pixels in vertical direction in focal plane
camera.Nxi = 99;%81;%%99 for HCIL experiment;
camera.Neta = 83;%31;%%83 for HCIL experiment;
camera.stacking = 1; % number of image for stacking
camera.exposure = 1; % exposure time in seconds for one image
camera.newDarkFrame = 1; % 1 for taking new dark frame, 0 for using existed dark frame
camera.centerLabView = [273, 293]; % only works for same binning, [500-x, y]
camera.center = [227,295];%[227,293];%[176, 314];%[254, 303];%[253, 303];%[252, 302];%[253, 303];%[252, 302];%[253, 303];%[253, 304];%[255, 305];%[254, 304];%[256, 307];%[255, 309];%[255, 310];%[256,310];%[256,311];%[257,311];%[257, 312];%[175,314];%[176, 334];%[176,334];%[178, 337];%[217, 257];%[219, 261];%[267, 275];%[266, 276];%[267, 274];%[269, 274];%[268, 277];%[194, 239];%[195, 238];%[255, 230];%[255, 231];%[232, 216];%[231, 220];%[232, 221];%[231, 224];%[232, 221]; % the center position of PSF on camera
camera.blockedXi = [-3, 3]; 
camera.blockedEta = [-3, 3]; % the blocked region by FPM, used for evaluating the background light and noise
camera.blockedCoordXi = floor((camera.blockedXi(1)-camera.visionXi(1))/(camera.visionXi(2)-camera.visionXi(1))*camera.Nxi): ...
    ceil((camera.blockedXi(2)-camera.visionXi(1))/(camera.visionXi(2)-camera.visionXi(1))*camera.Nxi);
camera.blockedCoordEta = floor((camera.blockedEta(1)-camera.visionEta(1))/(camera.visionEta(2)-camera.visionEta(1))*camera.Neta): ...
    ceil((camera.blockedEta(2)-camera.visionEta(1))/(camera.visionEta(2)-camera.visionEta(1))*camera.Neta);
camera.readoutstd = 12;%12; %0.01; % stand deviation of camera readout noise
camera.noise = 1; % 1 stand for that we put some virtual noises in the simulation

%% Initialize the parameters for the target, now only consider the monochromatic case
target.star = 1; % 1 for 'on', 0 for 'off'
target.planet = 0; % 1 for 'on', 0 for 'off'
target.broadBandControl = 1; % broadband control or monochromatic control
target.planetContrast = 1e-8; % the contrast of planet compared with star
target.starWavelength = 635e-9; % Unit: meters
target.starWavelengthBroad = 605e-9:10e-9:665e-9; % The broadband wavelengths
target.broadSampleNum = length(target.starWavelengthBroad); % The length of broadband wavelengths
target.planetWavelength = 648e-9; % Unit: meters
target.separation = 8; % Unit: Wavelength / Diameter (lambda / D)
target.normalization = 220.6292;%147.1332 for lab simulation; %1; % normalization factor for simulated images
target.flux = 1.92707e+09;%12.2802e+8;%13.1419e+8;%13.4576e+8;%13.8209e+8;%10.0209e+8;%10.0854e+8;%9.8533e+8;%5.4127e+8;%5.5863e+8;%6.5368e+8;%5.0031e+8;%3.6612e+8;%;%6.232e+8;%5.4321e+8;%4.4e+8;%4.87e+8;%4.8573e+8; %1.54382e+9;% laser_Power(54, 1); 4.556e+8;% %4.8573e+8;%5.1371e+8;%8e+8; % peak count of PSF per pixel per second

target.drift = 0; % 1 stands for the drift exists, 0 for no drift
target.NdriftMode = 18;
target.driftCoef = zeros(target.NdriftMode, 1); % the coefficient for each drift mode
target.driftCoefCollection = []; % record the change of drift coefficients
target.driftStd = 0.01e-9; % dirft standard deviation per frame, Unit: nm

if strcmpi(coronagraph.type, 'SPLC')
    if target.star
        target.EinStar = ones(coronagraph.Naperture, coronagraph.Naperture);
        coronagraph_help = coronagraph;
        coronagraph_help.FPmask = ones(size(coronagraph.FPmask));
        target_help = target;
        target_help.star = 1;
        target_help.planet = 0;
        target_help.normalization = 1;
        target.normalizationBroadband = ones(target.broadSampleNum, 1);
        for k = 1 : target.broadSampleNum
            target_help.starWavelength = target.starWavelengthBroad(k);
            [EfocalStar, EfocalPlanet, Ifocal] = opticalModel(target_help, DM, coronagraph_help, camera, zeros(DM.activeActNum, 1), zeros(DM.activeActNum, 1));
            target.normalizationBroadband(k) = max(max(Ifocal));
        end
        target.normalization = target.normalizationBroadband(4);
        clear coronagraph_help;
        clear target_help;
    end
    if target.planet
        minAngle = 0;
        maxAngle = target.separation * 2 * pi;
        gap = (maxAngle - minAngle) / coronagraph.Naperture;
        phase = ones(coronagraph.Naperture, coronagraph.Naperture) * diag(minAngle + gap : gap : maxAngle);
        target.EinPlanet = sqrt(target.planetContrast) * exp(1i * phase);
    end
    % define the drift modes
    xAperture = ((1 : coronagraph.Naperture) - 0.5 * (1 + coronagraph.Naperture)) / (0.5*coronagraph.Naperture);
    yAperture = xAperture;
    [XAperture, YAperture] = meshgrid(xAperture, yAperture);
    ndrift = 1;
    target.driftModes = zeros(coronagraph.Naperture, coronagraph.Naperture, target.NdriftMode);
    for k1 = 1 : 3
        for k2 = 1: 3
            for k3 = [0, 0.5]
                target.driftModes(:, :, ndrift) = cos(pi*(k1*XAperture + k2*YAperture + k3));
                ndrift = ndrift + 1;
            end
        end
    end
elseif strcmpi(coronagraph.type, 'SPC')
    if target.star
        target.EinStar = ones(DM.DMmesh(1), DM.DMmesh(2));
        target_help = target;
        target_help.star = 1;
        target_help.planet = 0;
        target_help.normalization = 1;
        target.normalizationBroadband = ones(target.broadSampleNum, 1);
        for k = 1 : target.broadSampleNum
            target_help.starWavelength = target.starWavelengthBroad(k);
            [EfocalStar, EfocalPlanet, Ifocal] = opticalModel(target_help, DM, coronagraph, camera, zeros(DM.activeActNum, 1), zeros(DM.activeActNum, 1));
            target.normalizationBroadband(k) = max(max(Ifocal));
        end
        target.normalization = target.normalizationBroadband(4);
        clear target_help;
    end
    if target.planet
        minAngle = 0;
        maxAngle = target.separation * 2 * pi;
        gap = (maxAngle - minAngle) / size(DM1shape, 2);
        phase = ones(size(DM1shape)) * diag(minAngle + gap : gap : maxAngle);
        target.EinPlanet = sqrt(target.planetContrast) * exp(1i * phase);
    end
    % define the drift modes
    xAperture = ((1 : DM.DMmesh(1)) - 0.5 * (1 + DM.DMmesh(1))) / (0.5*DM.DMmesh(1));
    yAperture = ((1 : DM.DMmesh(2)) - 0.5 * (1 + DM.DMmesh(2))) / (0.5*DM.DMmesh(2));
    [XAperture, YAperture] = meshgrid(xAperture, yAperture);
    ndrift = 1;
    target.driftModes = zeros(DM.DMmesh(1), DM.DMmesh(2), target.NdriftMode);
    for k1 = 1 : 3
        for k2 = 1: 3
            for k3 = [0, 0.5]
                target.driftModes(:, :, ndrift) = cos(pi*(k1*XAperture + k2*YAperture + k3));
                ndrift = ndrift + 1;
            end
        end
    end
end

if strcmpi(coronagraph.type, 'SPC')
    coronagraph.FPMmask = createMask(target, coronagraph, camera, 'wedge', ...
        'LR', [0, 0], [0, 0], [5, 11], 45); % generate the 2D focal plane mask
    coronagraph.FPMpixelIndex = find(coronagraph.FPMmask(:) == 1); % the pixel indices in the non-FPM-blocked region
    coronagraph.FPMpixelNum = length(coronagraph.FPMpixelIndex); % the number of pixels in the non-FPM-blocked region
end
%% Initialize the dark hole region
darkHole.type = 'wedge';%'wedge'; % the type(shape) of the dark hole regions - 'wedge' or 'box'
darkHole.side = 'LR';%'LR';% % the side where dark holes located - 'L', 'R' or 'LR'
darkHole.rangeX = [7, 10]; % used for 'box' dark hole only, unit - f * lambda / D
darkHole.rangeY = [-3, 3]; % used for 'box' dark hole only, unit - f * lambda / D
darkHole.rangeR = [2.5, 9];%[6, 10]; %[6, 11];%[5.5, 10.5]; % used for 'wedge' dark hole only, unit - f * lambda / D
darkHole.rangeAngle = 30; % used for 'wedge' dark hole only, ranged from 0 to 45, unit - degree
darkHole.mask = createMask(target, coronagraph, camera, darkHole.type, ...
    darkHole.side, darkHole.rangeX, darkHole.rangeY, darkHole.rangeR, darkHole.rangeAngle); % generate the 2D dark hole shape
darkHole.pixelIndex = find(darkHole.mask(:) == 1); % the pixel indices in the dark hole region
darkHole.pixelNum = length(darkHole.pixelIndex); % the number of pixels in the dark holes

%% Initialize the controllers
controller.type = 'EFC';%'EFC';%'speckleNulling';% % the controller type we use, 'EFC, 'speckleNulling', or 'robustLP'
controller.whichDM = 'both';%'both';%  % which DM we use for wavefront control, '1', '2' or 'both'
if strcmpi(controller.type, 'EFC')
    controller.alpha = 1e-6;%1.8e-8;%1e-6;%3e-8;%5e-6;%1e-5; %3e-8; % the Tikhonov regularization parameter for 'EFC'
    controller.adaptiveEFC = 0; % 1 stands for that we automatically choose regularization parameter, 0 stands for fixed regularization. It is a kind of greedy, so it is trapped by local minimum after some iterations.
end
if strcmpi(controller.type, 'robustLP')
    controller.gurobiPath = 'C:\gurobi651\win64\matlab';%'/Library/gurobi702/mac64/matlab';%'C:\gurobi651\win64\matlab';
    addpath(controller.gurobiPath);
    gurobi_setup();
end
if strcmpi(controller.type, 'speckleNulling')
    controller.whichDM = '1'; % only the first DM can be used for speckle nulling 
    if strcmpi(coronagraph.type, 'SPLC')
        Npsf = 7;
    elseif strcmpi(coronagraph.type, 'SPC')
        Npsf = 21;
    end
    if target.broadBandControl
        controller.PSF = zeros(Npsf, Npsf, target.broadSampleNum);
        for k = 1 : target.broadSampleNum
            target_perfect = target;
            target_perfect.starWavelength = target.starWavelengthBroad(k);
            DM_perfect = DM;
            camera_perfect = camera;
            coronagraph_perfect = coronagraph;
            target_perfect.star = 1;
            target_perfect.planet = 0;
            DM_perfect.noise = 0;
            camera_perfect.noise = 0;
            coronagraph_perfect.error = 0;
            if strcmpi(coronagraph.type, 'SPLC')
                coronagraph_perfect.FPmask = ones(size(coronagraph.FPmask));
            end
            [EnoPoke, ~, InoPoke]= opticalModel(target_perfect, DM_perfect, coronagraph_perfect, camera_perfect, zeros(size(DM.DM1command)), zeros(size(DM.DM2command)));
            controller.PSF(:, :, k) = InoPoke(42-(Npsf-1)/2:42+(Npsf-1)/2, 50-(Npsf-1)/2:50+(Npsf-1)/2);
        end
    else
        target_perfect = target;
        DM_perfect = DM;
        camera_perfect = camera;
        coronagraph_perfect = coronagraph;
        target_perfect.star = 1;
        target_perfect.planet = 0;
        DM_perfect.noise = 0;
        camera_perfect.noise = 0;
        coronagraph_perfect.error = 0;
        if strcmpi(coronagraph.type, 'SPLC')
            coronagraph_perfect.FPmask = ones(size(coronagraph.FPmask));
        end
        [EnoPoke, ~, InoPoke]= opticalModel(target_perfect, DM_perfect, coronagraph_perfect, camera_perfect, zeros(size(DM.DM1command)), zeros(size(DM.DM2command)));
        controller.PSF = InoPoke(42-(Npsf-1)/2:42+(Npsf-1)/2, 50-(Npsf-1)/2:50+(Npsf-1)/2);
    end
    clear coronagraph_perfect;
end
% controller.linearControllerType = 'cvxEnergyMin';%'SOSstrokeMin';%'cvxEnergyMin';%'SOSstrokeMin';%'cvxEnergyMin';%'SOSstrokeMin'; %'energyMin';

%% Initialize the estimators
estimator.type = 'batch';%'Kalman';%'EKF';%'perfect';% % the estimator type, 'perfect', 'batch', 'Kalman', 'EKF', 'UKF', 'overallKalman', 'preProcessKalman'
estimator.whichDM = '1';%'1'; % which DM we use for probing, '1' or '2'
estimator.NumImgPair = 2; % Used for 'batch' and 'Kalman'
estimator.NumImg = 1; % Used for 'EKF' or 'UKF', which not require pair-wise probing
estimator.linearProbe = 0; % 1 stands for only considering the linear part of DM probing, 0 stands for simulating the probing which include all the terms
estimator.EKFpairProbing = 0; % 1 stands for still using pair-wise probing, 0 stands for not
estimator.itrEKF = 10;%10;%3; % Used for 'EKF' only, IEKF iterations to make more accurate estimation
estimator.itrUKF = 10;%10; % Used for 'UKF' only, which has similar formula to IEKF
estimator.probeArea = [1, 17, -17, 17]; % Define the region in lambda / D
estimator.probeMethod = 'Empirical'; %'OptimalOffsets';% 'Empirical' or 'OptimalOffsets', choose the best probing offset to reduce state covariance
estimator.measuredAmp = 0; % 1 or 0, 1 stands for that we adjust the probing amplitude using measured images
estimator.saveData = 0; % 1 or 0, 1 stands for that we want to save the probing command and images for future run
estimator.stateStd0 = 1e-6; % the coefficient used to initialize the state covariance, used for Kalman filter and extended Kalman filter
estimator.processVarCoefficient = 3e-8;%3e-8;%0.05 * 1e-7;%0.05 * 1e-7;%0.01 * 1e-7 for EKF 2 pair and UKF 2 images%0.01 * 1e-8; for EKF 1 pair and 1 image%0.3 * 1e-7 for lab% the coefficient used for compute the process covariance noise, used for Kalman filter and extended Kalman filter
estimator.observationVarCoefficient = 1e-16;%3e-14;%3e-15;%1e-14;%6e-18;%6e-18; % the coefficient used for compute the observation covariance noise matrix
estimator.incoherentStd0 = 1e-5;%0.1e-7; % the std of incoherent process noise used for 'EKF'
estimator.incoherentStd = 1e-10;%1e-10;
estimatorBatch = estimator;
estimatorBatch.type = 'batch';
estimatorBatch.NumImgPair = 2;
%% Initialize the linear system identification algorithm
train.switch = 0; % 1 stands for online EM-algorithm is on, otherwise it is 'off'
if train.switch == 1
    train.batchsize = 10; % the batch size for online learning
    train.u = zeros(DM.activeActNum, train.batchsize); % control commands as inputs for E-M algorithm
    train.y = zeros(darkHole.pixelNum, estimator.NumImgPair, train.batchsize); % difference image as outputs for E-M algorithm
    train.G = zeros(darkHole.pixelNum, DM.activeActNum); % the learned Jacobian matrix
    train.Q = repmat(3e-9 * eye(2), [1, 1, darkHole.pixelNum]);%zeros(darkHole.pixelNum, 2, 2); % the process noise matrices
    train.R = repmat(3e-13 * eye(2), [1, 1, darkHole.pixelNum]);%zeros(darkHole.pixelNum, estimator.NumImgPair, estimator.NumImgPair); % the readout noise matrices
    train.x0 = zeros(darkHole.pixelNum, 2); % the initial state vector
    train.P0 = zeros(2, 2, darkHole.pixelNum); % the initial state covariance matrix
    train.uProbe = zeros(DM.activeActNum, estimator.NumImgPair, train.batchsize); % the probing control commands
    train.delta1 = 1e-5;%0.1; % the tuning parameters for the online learning algorithm
    train.delta2 = 1e-1;%1; % the tuning parameters for the online learning algorithm
    train.EMitr = 1; % the E-M iterations used for the problem
end

%% Initialize the structure for saving data
%-------------------------- Sepckle Nulling -------------------------------
if strcmpi(controller.type, 'speckleNulling')
    data.controllerType = controller.type;
    if target.broadBandControl
        data.measuredContrastAverage = zeros(Nitr, 1); % the measured average contrast in the dark holes (after wavefront correction)
        data.measuredContrastMax = zeros(Nitr, 1); % the measured max contrast in the dark holes (after wavefront correction)
        data.measuredContrastStd = zeros(Nitr, 1); % the std of measured contrast in the dark holes (after wavefront correction)
        data.DMcommand = zeros(2 * DM.activeActNum, Nitr); % the DM control command for each iteration
        data.backgroundAverage = zeros(Nitr, 1); % the average contrast of background
        data.backgroundStd = zeros(Nitr, 1); % the std deviation of the backgroud
        data.probeImage = zeros(camera.Neta, camera.Nxi, target.broadSampleNum, 4, Nitr); % probe images during speckle nulling control
        data.I = zeros(camera.Neta, camera.Nxi,target.broadSampleNum, Nitr); % used to save the focal images after each control iteration
    else
        data.measuredContrastAverage = zeros(Nitr, 1); % the measured average contrast in the dark holes (after wavefront correction)
        data.measuredContrastMax = zeros(Nitr, 1); % the measured max contrast in the dark holes (after wavefront correction)
        data.measuredContrastStd = zeros(Nitr, 1); % the std of measured contrast in the dark holes (after wavefront correction)
        data.DMcommand = zeros(2 * DM.activeActNum, Nitr); % the DM control command for each iteration
        data.backgroundAverage = zeros(Nitr, 1); % the average contrast of background
        data.backgroundStd = zeros(Nitr, 1); % the std deviation of the backgroud
        data.probeImage = zeros(camera.Neta, camera.Nxi, 4, Nitr); % probe images during speckle nulling control
        data.I = zeros(camera.Neta, camera.Nxi, Nitr); % used to save the focal images after each control iteration
    end
elseif strcmpi(controller.type, 'EFC')
%-------------------------------- EFC -------------------------------------
    data.controllerType = [controller.type, estimator.type];
    data.itr = 0;
    if target.broadBandControl
        data.I0 = zeros(camera.Neta, camera.Nxi, target.broadSampleNum); % used to save the original focal images
        data.I = zeros(camera.Neta, camera.Nxi, target.broadSampleNum, Nitr); % used to save the focal images after each control iteration
        data.EfocalEst = zeros(darkHole.pixelNum, target.broadSampleNum, Nitr); % the estimated coherent electric field at each iteration
        data.IincoEst = zeros(darkHole.pixelNum, target.broadSampleNum, Nitr); % the estimated incoherent light itensity
        data.EfocalEstPerfect = zeros(darkHole.pixelNum, target.broadSampleNum, Nitr); % the estimated coherent electric field at each iteration
        data.IincoEstPerfect = zeros(darkHole.pixelNum, target.broadSampleNum, Nitr); % the estimated incoherent light itensity
        data.measuredContrastAverage = zeros(target.broadSampleNum, Nitr); % the measured average contrast in the dark holes (after wavefront correction)
        data.estimatedContrastAverage = zeros(target.broadSampleNum, Nitr); % the estimated average contrast in the dark holes (before wavefront correction)
        data.estimatedIncoherentAverage = zeros(target.broadSampleNum, Nitr); % the estimated average incoherent contrast in the dark holes (before wavefront correction)
        data.measuredContrastMax = zeros(target.broadSampleNum, Nitr); % the measured max contrast in the dark holes (after wavefront correction)
        data.estimatedContrastMax = zeros(target.broadSampleNum, Nitr); % the estimated max contrast in the dark holes (before wavefront correction)
        data.measuredContrastStd = zeros(target.broadSampleNum, Nitr); % the std of measured contrast in the dark holes (after wavefront correction)
        data.estimatedContrastStd = zeros(target.broadSampleNum, Nitr); % the std of estimated contrast in the dark holes (before wavefront correction)
        data.DMcommand = zeros(2 * DM.activeActNum, Nitr); % the DM control command for each iteration
        data.backgroundAverage = zeros(target.broadSampleNum, Nitr); % the average contrast of background
        data.backgroundStd = zeros(target.broadSampleNum, Nitr); % the std deviation of the backgroud
        data.probeContrast = zeros(Nitr, 1); % the probe contrast
        data.y = zeros(darkHole.pixelNum, estimator.NumImgPair, Nitr); % the difference images
        data.yBroadband = zeros(darkHole.pixelNum, estimator.NumImgPair, target.broadSampleNum, Nitr); % the difference images
        switch lower(estimator.type)
            case {'perfect'}
            case {'kalman', 'batch'}
%                 data.P = zeros(2, 2, darkHole.pixelNum, Nitr);
%                 data.Pbroadband = zeros(2, 2, darkHole.pixelNum, target.broadSampleNum, Nitr);
                data.P = zeros(2, 2, darkHole.pixelNum, target.broadSampleNum, Nitr);
                data.uProbe = zeros(DM.activeActNum, estimator.NumImgPair, Nitr); % the probe shapes
            case {'ekf', 'ukf'}
%                 data.P = zeros(3, 3, darkHole.pixelNum, Nitr);
%                 data.Pbroadband = zeros(3, 3, darkHole.pixelNum, target.broadSampleNum, Nitr);
                data.P = zeros(3, 3, darkHole.pixelNum, target.broadSampleNum, Nitr);
                data.uProbe = zeros(DM.activeActNum, estimator.NumImg, Nitr); % the probe shapes
            otherwise
                disp('The estimator type can only be batch, kalman, overallkalman, ekf or ukf');
                return;
        end
        if estimator.saveData
            data.imageSet = cell(Nitr, 1); % the image set used for estimation
            data.probeSet = cell(Nitr, 1); % the command set for probing
        end
        if strcmpi(simOrLab, 'simulation')
            data.coherentErr = zeros(target.broadSampleNum, Nitr);
            data.incoherentErr = zeros(target.broadSampleNum, Nitr);
        end
        data.contrast0 = zeros(target.broadSampleNum, 1);
        data.contrast0Max = zeros(target.broadSampleNum, 1);
        data.contrast0Std = zeros(target.broadSampleNum, 1);
        data.estimatedContrastAverage0 = zeros(target.broadSampleNum, 1);
        data.estimatedContrastMax0 = zeros(target.broadSampleNum, 1);
        data.estimatedContrastStd0 = zeros(target.broadSampleNum, 1);
        data.estimatedIncoherentAverage0 = zeros(target.broadSampleNum, 1);
        data.EfocalEst0 = zeros(darkHole.pixelNum, target.broadSampleNum);
        data.IincoEst0 = zeros(darkHole.pixelNum, target.broadSampleNum);
    else
        data.I = zeros(camera.Neta, camera.Nxi, Nitr); % used to save the focal images after each control iteration
        data.EfocalEst = zeros(darkHole.pixelNum, Nitr); % the estimated coherent electric field at each iteration
        data.IincoEst = zeros(darkHole.pixelNum, Nitr); % the estimated incoherent light itensity
        data.measuredContrastAverage = zeros(Nitr, 1); % the measured average contrast in the dark holes (after wavefront correction)
        data.estimatedContrastAverage = zeros(Nitr, 1); % the estimated average contrast in the dark holes (before wavefront correction)
        data.estimatedIncoherentAverage = zeros(Nitr, 1); % the estimated average incoherent contrast in the dark holes (before wavefront correction)
        data.measuredContrastMax = zeros(Nitr, 1); % the measured max contrast in the dark holes (after wavefront correction)
        data.estimatedContrastMax = zeros(Nitr, 1); % the estimated max contrast in the dark holes (before wavefront correction)
        data.measuredContrastStd = zeros(Nitr, 1); % the std of measured contrast in the dark holes (after wavefront correction)
        data.estimatedContrastStd = zeros(Nitr, 1); % the std of estimated contrast in the dark holes (before wavefront correction)
        data.DMcommand = zeros(2 * DM.activeActNum, Nitr); % the DM control command for each iteration
        data.backgroundAverage = zeros(Nitr, 1); % the average contrast of background
        data.backgroundStd = zeros(Nitr, 1); % the std deviation of the backgroud
        data.probeContrast = zeros(Nitr, 1); % the probe contrast
        data.y = zeros(darkHole.pixelNum, estimator.NumImgPair, Nitr); % the difference images
        switch lower(estimator.type)
            case {'perfect'}
            case {'kalman', 'batch'}
                data.P = zeros(2, 2, darkHole.pixelNum, Nitr);
                data.uProbe = zeros(DM.activeActNum, estimator.NumImgPair, Nitr); % the probe shapes
            case {'ekf', 'ukf'}
                data.P = zeros(3, 3, darkHole.pixelNum, Nitr);
                data.uProbe = zeros(DM.activeActNum, estimator.NumImg, Nitr); % the probe shapes
            otherwise
                disp('The estimator type can only be batch, kalman, overallkalman, ekf or ukf');
                return;
        end
        if estimator.saveData
            data.imageSet = cell(Nitr, 1); % the image set used for estimation
            data.probeSet = cell(Nitr, 1); % the command set for probing
        end
        if strcmpi(simOrLab, 'simulation')
            data.coherentErr = zeros(Nitr, 1);
            data.incoherentErr = zeros(Nitr, 1);
        end
    end
end

%% Initialize the hardware driver if we are running experiment
if (strcmpi(simOrLab, 'lab')) % if conducting experiment in lab, initialize DM and camera drivers
%     DM.DM1bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_CLOSED_LOOP_200nm_VOLTAGES.txt','-ascii');
    DM.DM1bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW004#14_CLOSED_LOOP_200nm_Voltages_DM#1.txt','-ascii');
    DM.DM1bias = DM.DM1bias(1 : DM.activeActNum); % flatten map voltages in volts on DM1
%     DM.DM2bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#014_CLOSED_LOOP_200nm_Voltages.txt','-ascii');
    DM.DM2bias = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW018#40_CLOSED_LOOP_200nm_Voltages_DM#2.txt','-ascii');
    DM.DM2bias = DM.DM2bias(1 : DM.activeActNum); % flatten map voltages in volts on DM2
    initializeDM(DM);
    camera = initializeCamera(camera);
end