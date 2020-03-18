%% initialization function of broadband wavefront control
% Revised from Initialization.m by He Sun on Jul. 30, 2019
%
% Initialize the setup of the simulation or the experiment, including the optical model, DM
% model and so on;
%

%% Sec #1 - Initialize the path, should change for different computers
computerID = 'walle_w' % 'ultron', 'hesun', or 'hesunLaptop'
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
        folder.dataLibrary = 'C:\Lab\FPWCmatlab\dataLibrary\20191015';
        folder.LOWFS = 'C:\Lab\FPWCmatlab\LOWFS';
        folder.python = 'C:\Lab\FPWCpy\active_estimation';
        folder.IFS = 'C:\Lab\FPWCmatlab\IFS';
    case 'walle_w'
        folder.main = 'C:\Users\sfr\Documents\HCIL_pton\FPWCmatlab';
        folder.optics = 'C:\Users\sfr\Documents\HCIL_pton\FPWCmatlab\opticalModel';
        folder.DM = 'C:\Users\sfr\Documents\HCIL_pton\FPWCmatlab\DMmodel';
        folder.SSM = 'C:\Users\sfr\Documents\HCIL_pton\FPWCmatlab\stateSpaceModel';
        folder.controller = 'C:\Users\sfr\Documents\HCIL_pton\FPWCmatlab\controller';
        folder.estimator = 'C:\Users\sfr\Documents\HCIL_pton\FPWCmatlab\estimator';
        folder.hardware = 'C:\Users\sfr\Documents\HCIL_pton\FPWCmatlab\hardware';
        folder.dataLibrary = 'C:\Users\sfr\Documents\HCIL_pton\FPWCmatlab\dataLibrary\20200317';
        folder.LOWFS = 'C:\Users\sfr\Documents\HCIL_pton\FPWCmatlab\LOWFS';
%         folder.python = 'C:\Lab\FPWCpy\active_estimation';
        folder.IFS = 'C:\Users\sfr\Documents\HCIL_pton\FPWCmatlab\IFS';
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
addpath(genpath(folder.optics));
addpath(folder.DM);
addpath(folder.dataLibrary);
addpath(folder.SSM);
addpath(folder.controller);
addpath(folder.estimator);
addpath(folder.hardware);
addpath(folder.IFS);

% addpath(folder.LOWFS);

%% The parameters for the optical model
% including light source, DM model, coronagraph shape and distances

%% Sec #2 - Initialize the parameter for the DM
DM.model = 'influencingFunction'; % 'influencingFuntion' (or 'FEM', 'neuralNet')
DM.DMmesh = [442, 442]; % The pixel number in each direction should better be even, [442, 442] is a good choice to recover surface shape using linear superposition
DM.Nact = 34; % number of actuators in one direction
DM.pitch = 282.85e-06;%280.15e-06;%280.64e-6;%286.17e-6;%282.64e-6;% pitch size of DM actuator in meters, 301e-6 for SPLC, 282.7e-6 for SPC
DM.widthDM = DM.pitch * DM.Nact; % DM width in meters, usually the DM is square
DM.DM1gain = 5.06e-9 * ones(DM.Nact, DM.Nact); %6.27e-9 * ones(DM.Nact, DM.Nact);
DM.DM2gain = 6.27e-9 * ones(DM.Nact, DM.Nact); % the DM gain (voltage to height) of each actuator, unit: meter / volt
% temp = load('DM1gain.mat');
% DM.DM1gain = temp.DM1gain;
% temp = load('DM2gain.mat');
% DM.DM2gain = temp.DM2gain;

DM.zDM1toDM2 = 0.33697;%0.23;%0.42545;%simulation shows 0.22 may be the best, we use 0.23 for SPLC simulation % distance from  DM1 to DM2 in meters
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
DM.DMperfect.DM1gain = 5.06e-9 * ones(DM.Nact, DM.Nact);%5.06e-9 * ones(DM.Nact, DM.Nact);%5.06e-9 * (ones(DM.Nact, DM.Nact) + 0.8 * (rand(DM.Nact, DM.Nact)-0.5));% %6.27e-9 * ones(DM.Nact, DM.Nact);
DM.DMperfect.DM2gain = 6.27e-9 * ones(DM.Nact, DM.Nact);%6.27e-9 * ones(DM.Nact, DM.Nact); % the DM gain (voltage to height) of each actuator, unit: meter / volt

%% Sec #3 - Initialize the coronagraph instrument layout
coronagraph.type = 'SPC';%'VORTEX';%'SPLC';%

coronagraph.SPwidth = 0.01; % width of shaped pupil mask in meters
coronagraph.Nsp = round(coronagraph.SPwidth / DM.widthDM * DM.DMmesh(1)); % number of pixels in one direction of shaped pupil mask matrix
coronagraph.zDM2toSP = 0.5334;%0.51308;%0.581; % distance from DM2 to shaped pupil mask in meters
coronagraph.focalLength = 1.1675;%1.1729;%1.1697;%1.1638;%1.1786;%1.1630;%1.1676;%1.1642;%1.13;%1.1528;%1.141;%1.1379; %1.1652;%1.85; % focal length in meters
coronagraph.SPshape = load([folder.optics '/SPs/ripple3_256x256_ideal_undersized.txt']);
% coronagraph.SPshape = MakeMaskEllipse12b(coronagraph.Nsp/2, folder);
% coronagraph.FPM = ones(camera.Nxi, camera.Neta);
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
%% Sec #4 - Initialize the parameters for camera
camera.name = 'QSI'; %'Starlight'; % 
if strcmpi(camera.name, 'QSI')
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
    camera.exposure0 = 1; % the exposure time used for non-probe image
    camera.newDarkFrame = 1; % 1 for taking new dark frame, 0 for using existed dark frame
    camera.centerLabView = [273, 293]; % only works for same binning, [500-x, y]
    camera.center = [157, 310];%[156, 308];%[170, 302];%[168, 302];%[167, 301];%[168, 299];%[170, 299];%[186, 294];%[193, 295];%[196, 320];%[199,321];% [210, 290];%[214, 288];%[206, 253];%[196, 262];%[196, 263];%[198, 263];%[205, 266];%[209, 286];%[217, 295];%[163, 234];%[167, 278];%[171, 300];%[140, 282];%[138, 287];%[140, 286];%[130,296];%[135, 302];%[112, 308];%[113, 310];%[113, 304];%[114, 303];%[112, 302];%[113, 302];%[113, 304];%[117, 302];%[118, 301];%[140, 264];%[142, 264];%[279, 243];%[280, 243];%[277, 248];%[250, 264];%[249, 285];%[250, 284];%[245, 281];%[245, 280];%[227,295];%[227,293];%[176, 314];%[254, 303];%[253, 303];%[252, 302];%[253, 303];%[252, 302];%[253, 303];%[253, 304];%[255, 305];%[254, 304];%[256, 307];%[255, 309];%[255, 310];%[256,310];%[256,311];%[257,311];%[257, 312];%[175,314];%[176, 334];%[176,334];%[178, 337];%[217, 257];%[219, 261];%[267, 275];%[266, 276];%[267, 274];%[269, 274];%[268, 277];%[194, 239];%[195, 238];%[255, 230];%[255, 231];%[232, 216];%[231, 220];%[232, 221];%[231, 224];%[232, 221]; % the center position of PSF on camera
    camera.blockedXi = [-3, 3]; 
    camera.blockedEta = [-3, 3]; % the blocked region by FPM, used for evaluating the background light and noise
    camera.blockedCoordXi = floor((camera.blockedXi(1)-camera.visionXi(1))/(camera.visionXi(2)-camera.visionXi(1))*camera.Nxi): ...
        ceil((camera.blockedXi(2)-camera.visionXi(1))/(camera.visionXi(2)-camera.visionXi(1))*camera.Nxi);
    camera.blockedCoordEta = floor((camera.blockedEta(1)-camera.visionEta(1))/(camera.visionEta(2)-camera.visionEta(1))*camera.Neta): ...
        ceil((camera.blockedEta(2)-camera.visionEta(1))/(camera.visionEta(2)-camera.visionEta(1))*camera.Neta);
    camera.readoutstd = 12;%12; %0.01; % stand deviation of camera readout noise
    camera.noise = 1; % 1 stand for that we put some virtual noises in the simulation
elseif strcmpi(camera.name, 'Starlight')
    camera.pitch = 85.72e-6; % pitch size of the camera pixel in meters
    camera.binXi = 1; % pixel binning in x direction
    camera.binEta = 1; % pixel binning in y direction
    camera.imageSize = [500, 500]; % the total image size in pixels get from camera
    camera.startPosition = [0, 0]; % the starting camera pixel in taking images
    camera.visionXi = [-12, 12]; % field of vision in horizontal direction, defined by how many f*lambda/D
    camera.visionEta = [-10, 10]; % field of vision in vertical direction, defined by how many f*lambda/D
    camera.Nxi = 45;%65;%99 for HCIL experiment;
    camera.Neta = 33;%57;%%83 for HCIL experiment;
    camera.Nxi_half = 0.5 * (camera.Nxi-1);
    camera.Neta_half = 0.5 * (camera.Neta-1);
    camera.stacking = 1; % number of image for stacking
    camera.exposure = 10; % exposure time in seconds for one image
    camera.center = [55, 54];%[55, 56];%[54, 55];%[53, 55];%[54, 54];%[52, 54];%[52,54];%[52, 53];%[186, 294];
    camera.readoutstd = 12;%12; %0.01; % stand deviation of camera readout noise
    camera.noise = 1; % 1 stand for that we put some virtual noises in the simulation
    camera.darkFrame = 1263;%1.2789e+03;
    camera.philens = 26.56505117707799;
    camera.IFSlam = 1e-9 * [603.04640033, 609.1856823 , 615.3874649 , 621.65238443, ...
       627.98108364, 634.37421183, 640.83242493, 647.35638552, ...
       653.94676295, 660.60423337, 667.32947981, 674.12319227, ...
       680.98606776, 687.91881039, 694.92213144, 701.99674943, ...
       709.14339019, 716.36278695];
   
    camera.IFSflux = [26791938.7468077,27705595.4467567,29493304.1418635,30506072.0489820,29914566.4742719,29722596.8133114,29292508.1059290,29363886.2721767,29589173.8275212,30451860.8533752,30898262.6542102,30932912.7955735,30974470.4985311,30195761.4695486,29485110.1558805,26978044.1581202,24403012.5128366,20578861.7678486];
    % First one post spatial filter: [19256720.4896028,19757418.9034247,21285463.0910513,20956079.5165011,20727343.0204110,20413054.2595488,20799266.4711976,21080280.3552267,21045859.8496449,21755968.5129265,21817239.0280621,21723794.6199019,21928323.8624936,21883157.0369763,21561623.6919506,21292976.4909803,19995660.6773757,18352879.7180432];
    % Last one pre- new spatial filter:%[278340.135473760,348386.194342463,429315.443473963,581072.447845133,670737.526315581,724198.935853289,722577.234219789,728424.375886114,724278.554542832,680952.027787953,672273.074570652,569703.505286769,543710.324409378,518210.962868499,478605.379451082,384055.651038802,332053.525174727,308661.594644962];
    
%     1e6 * [1.1905, 1.5008, 1.8965, 2.3415, 2.5271, 2.4440, ...
%         2.3371, 2.0909, 2.0608, 1.8734, 1.9277, 1.9013, 1.9853, 1.7657, ...
%         1.6562, 1.5770, 1.4459, 1.3117]; %[1100217.00308190,1526333.79083406,1893574.46587861,2009284.78083710,2300614.42936023,2226927.31828514,2115421.24391296,2142733.13148029,1915865.88429213,1901940.56347266,2118354.67601292,2061433.13110379,1873361.43235083,1831964.11345055,1710662.01492352,1512577.91931564,1252198.32123141,1160692.24759962];%
    camera.IFSlamSam = 3:3:15;
    
%     camera.IFSflux = 1e6 * [0.1152, 0.1503, 0.2618, 0.4964, 0.9316, 1.0780, ...
%         0.7908, 0.4035, 0.2086, 0.1696, 0.1685, 0.1619, 0.1726, 0.1921, ...
%         0.1639, 0.1746, 0.1468, 0.1368] * 20.28;
%     camera.IFSlamSam = 6;
   %% change the focal length if we are using IFS
   coronagraph.focalLength = 2.59;
   coronagraph.coronagraph_perfect.focalLength = coronagraph.focalLength;
end

%% Sec #5 - Initialize the parameters for the target, now only consider the monochromatic case
target.channel = [3, 4, 6, 7, 8, 9, 10];
if strcmpi(camera.name, 'QSI') && strcmpi(simOrLab,'lab')
    % Initialize filter wheel for QSI case
    target.laser = serial('COM12','BaudRate',115200,'Terminator','CR'); %initialize filter wheel for QSI case
    fopen(target.laser)
    
    target.starWavelengthBroad = 1e-9 * [600,620,640,650,670,694.3,720];
    target.fluxBroadband = [4.4408e5, 7.1824e5, 6.4512e5, 6.6056e5, 5.2720e5, 3.9396e5];%[1.283e7];
    % target.channel = [10];
elseif strcmpi(camera.name, 'Starlight')
    target.starWavelengthBroad = camera.IFSlam(camera.IFSlamSam); %
    % target.starWavelengthBroad = 1e-9 * [635]; %
    
%     target.broadSampleNum = length( target.starWavelengthBroad); % The length of broadband wavelengths
    target.fluxBroadband = camera.IFSflux(camera.IFSlamSam);
end

target.star = 1; % 1 for 'on', 0 for 'off'
target.planet = 0; % 1 for 'on', 0 for 'off'
target.broadBandControl = 1; % broadband control or monochromatic control
target.planetContrast = 1e-8; % the contrast of planet compared with star
target.starWavelength = 660e-9; %658e-9;% Unit: meters

% % target.starWavelengthBroad = 1e-9 * [600,620,640,650,670,694.3,720]; %[550,577,600,620,632.8,640,650,670,694.3,720,740]; % The broadband wavelengths
% target.starWavelengthBroad = camera.IFSlam(camera.IFSlamSam); %
% % target.starWavelengthBroad = 1e-9 * [635]; %

target.broadSampleNum = length(target.starWavelengthBroad); % The length of broadband wavelengths
target.planetWavelength = 648e-9; % Unit: meters
target.separation = 8; % Unit: Wavelength / Diameter (lambda / D)
target.normalization = 220.6292;%147.1332 for lab simulation; %1; % normalization factor for simulated images
target.flux = 1.5474e+09;%1.4550e+09;%1.57911e+9;%1.84538e+9;%1.77977e+9;%1.6229e+9;%1.51435e+9;%1.65766e+9;%1.54116e+9;%1.81802e+9;%1.80278e+9;%1.724e+9;%1.659e+9;%1.8132e+9;%1.92707e+09;%12.2802e+8;%13.1419e+8;%13.4576e+8;%13.8209e+8;%10.0209e+8;%10.0854e+8;%9.8533e+8;%5.4127e+8;%5.5863e+8;%6.5368e+8;%5.0031e+8;%3.6612e+8;%;%6.232e+8;%5.4321e+8;%4.4e+8;%4.87e+8;%4.8573e+8; %1.54382e+9;% laser_Power(54, 1); 4.556e+8;% %4.8573e+8;%5.1371e+8;%8e+8; % peak count of PSF per pixel per second
% % target.fluxBroadband = [6.07e6, 6.92e6, 6.7733e6, 7.9e6, 6.9333e6, 5.1533e6, 6.84e6];% for QSI
% % target.fluxBroadband = [3.1840e5, 4.7700e5, 5.0067e5, 5.5333e5, 4.6467e5, 3.0183e5]; % for starlight express IFS
% target.fluxBroadband = camera.IFSflux(camera.IFSlamSam);
% % target.fluxBroadband = [6.84e6];
% % target.fluxBroadband = [1.5474e+09];
% 
% 
% % target.fluxBroadband = [1.1e7];%[1.283e7];
% % target.fluxBroadband = [4.4408e5, 7.1824e5, 6.4512e5, 6.6056e5, 5.2720e5, 3.9396e5];%[1.283e7];


target.drift = 0; % 1 stands for the drift exists, 0 for no drift
target.NdriftMode = 18;
target.driftCoef = zeros(target.NdriftMode, 1); % the coefficient for each drift mode
target.driftCoefCollection = []; % record the change of drift coefficients
target.driftStd = 0.01e-9; % dirft standard deviation per frame, Unit: nm

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
    target.normalization = target.normalizationBroadband(1);
    clear target_help;
end
if target.planet
    minAngle = 0;
    maxAngle = target.separation * 2 * pi;
    gap = (maxAngle - minAngle) / size(DM1shape, 2);
    phase = ones(size(DM1shape)) * diag(minAngle + gap : gap : maxAngle);
    target.EinPlanet = sqrt(target.planetContrast) * exp(1i * phase);
end

if strcmpi(coronagraph.type, 'SPC')
    coronagraph.FPMmask = createMask(target, coronagraph, camera, 'wedge', ...
        'LR', [0, 0], [0, 0], [5, 11], 45); % generate the 2D focal plane mask
    coronagraph.FPMpixelIndex = find(coronagraph.FPMmask(:) == 1); % the pixel indices in the non-FPM-blocked region
    coronagraph.FPMpixelNum = length(coronagraph.FPMpixelIndex); % the number of pixels in the non-FPM-blocked region
end
%% Sec #6 - Initialize the dark hole region
darkHole.type = 'wedge';%'box';%'circ';% % the type(shape) of the dark hole regions - 'wedge' or 'box' or 'circ'
darkHole.side = 'LR';%'LR';%'R';% % the side where dark holes located - 'L', 'R' or 'LR'
darkHole.rangeX = [7, 10]; % used for 'box' dark hole only, unit - f * lambda / D
darkHole.rangeY = [-3, 3]; % used for 'box' dark hole only, unit - f * lambda / D
darkHole.rangeR = [6, 9];%[2.5, 9] for SPLC;%[6, 10]; %[5.5, 10.5]; % used for 'wedge' dark hole only, unit - f * lambda / D
darkHole.rangeAngle = 30;%42.5;% 30 for SPLC; % used for 'wedge' dark hole only, ranged from 0 to 45, unit - degree

darkHole.mask = createMask(target, coronagraph, camera, darkHole.type, ...
    darkHole.side, darkHole.rangeX, darkHole.rangeY, darkHole.rangeR, darkHole.rangeAngle); % generate the 2D dark hole shape
darkHole.pixelIndex = find(darkHole.mask(:) == 1); % the pixel indices in the dark hole region
darkHole.pixelNum = length(darkHole.pixelIndex); % the number of pixels in the dark holes

%% Sec #7 - Initialize the controllers
controller.type = 'EFC';%%'speckleNulling';% % the controller type we use, 'EFC, 'speckleNulling', or 'robustLP'
controller.whichDM = 'both';%'1';%  % which DM we use for wavefront control, '1', '2' or 'both'
controller.alpha = 1e-4;%3e-5;%1e-4;%3e-8;%3e-5;%5e-7;%%1.8e-8;%1e-6;%3e-8;%5e-6;%1e-5; %3e-8; % the Tikhonov regularization parameter for 'EFC'

%% Sec #8 - Initialize the estimators
estimator.type = 'batch';%'EKF';%'Kalman';%%'perfect';% % the estimator type, 'perfect', 'batch', 'Kalman', 'EKF', 'UKF', 'overallKalman', 'preProcessKalman'
estimator.whichDM = '1';%'both';% % which DM we use for probing, '1', '2' or 'both'
estimator.NumImgPair = 2; % Used when EKFpairProbing is 1
estimator.NumImg = 4; % Used when EKFpairProbing is 0
estimator.linearProbe = 1;%1; % 1 stands for only considering the linear part of DM probing, 0 stands for simulating the probing which include all the terms
estimator.nonProbeImage = 0;
estimator.EKFpairProbing = 1; % 1 stands for still using pair-wise probing, 0 stands for not
estimator.EKFincoherent = 0; % 1 stands for estimating incoherent in EKF, 0 stands for assuming no incoherent light
estimator.optimized_probe = 0;
estimator.itrEKF = 10;%10;%10;%3; % Used for 'EKF' only, IEKF iterations to make more accurate estimation
estimator.itrUKF = 10;%10; % Used for 'UKF' only, which has similar formula to IEKF
estimator.probeArea = [1, 17, -17, 17]; %[0, 17, -17, 17]; % Define the region in lambda / D
estimator.probeMethod = 'Empirical'; %'OptimalOffsets';% 'Empirical' or 'OptimalOffsets', choose the best probing offset to reduce state covariance
estimator.measuredAmp = 0;%0;%1; % 1 or 0, 1 stands for that we adjust the probing amplitude using measured images
estimator.saveData = 0; % 1 or 0, 1 stands for that we want to save the probing command and images for future run
estimator.stateStd0 = 1e-5;%7e-6;%1e-6 % the coefficient used to initialize the state covariance, used for Kalman filter and extended Kalman filter
estimator.processVarCoefficient = 5e-9;%6e-9;%3e-8;% 3e-9 for physics model;%3e-8;%0.05 * 1e-7;%0.05 * 1e-7;%0.01 * 1e-7 for EKF 2 pair and UKF 2 images%0.01 * 1e-8; for EKF 1 pair and 1 image%0.3 * 1e-7 for lab% the coefficient used for compute the process covariance noise, used for Kalman filter and extended Kalman filter
estimator.processVarCoefficient2 = 2e-10;% 1e-9;%
estimator.observationVarCoefficient = 5e-17;% 1e-14;%3e-14;%1e-16;%3e-14;%3e-15;%1e-14;%6e-18;%6e-18; % the coefficient used for compute the observation covariance noise matrix
estimator.observationVarCoefficient1 = 1.0 / (target.flux * camera.exposure); % scaling coefficient for camera possion noises
estimator.observationVarCoefficient2 = 0.8e-14;% 2e-14 for physcis model;%3.68e-13;%5e-14;%7e-13;
estimator.observationVarCoefficient3 = 0.0015;%0.022;%0.009;%0.022;% for SPC, 0.009;% for SPLC 0.008;% for SPC aberrated
estimator.observationVarCoefficient10 = 1.0 / (target.flux);
estimator.observationVarCoefficient0 = 5e-17; % the readout noise parameter normalized by time
% estimator.processVarCoefficient = 5.16e-9;%3e-9;%3e-8;%3e-8;%0.05 * 1e-7;%0.05 * 1e-7;%0.01 * 1e-7 for EKF 2 pair and UKF 2 images%0.01 * 1e-8; for EKF 1 pair and 1 image%0.3 * 1e-7 for lab% the coefficient used for compute the process covariance noise, used for Kalman filter and extended Kalman filter
% estimator.processVarCoefficient2 = 1.4e-9;%1e-10;%6.6943e-10;
% estimator.observationVarCoefficient = 1e-14;%3e-14;%1e-16;%3e-14;%3e-15;%1e-14;%6e-18;%6e-18; % the coefficient used for compute the observation covariance noise matrix
% estimator.observationVarCoefficient2 = 3.68e-13;%5e-14;%7e-13;
estimator.incoherentStd0 = 1e-13;%0.1e-7; % the std of incoherent process noise used for 'EKF'
estimator.incoherentStd = 1e-9;%1e-10;

estimatorBatch = estimator;
estimatorBatch.type = 'batch';
estimatorBatch.whichDM = '1';
estimatorBatch.NumImgPair = 2;
estimatorBatch.EKFpairProbing = 1;
estimatorBatch.adaptive_exposure = 0;
estimatorBatch.activeSensing = 0;
% active sensing part
estimator.activeSensing = 0;

%% Sec #9 - Initialize the structure for saving data

if strcmpi(controller.type, 'EFC')
%-------------------------------- EFC -------------------------------------
    data.controllerType = [controller.type, estimator.type];
    data.itr = 0;
    if target.broadBandControl
        data.IFSimage0 = zeros(1024, 1024);
        data.IFSimage = zeros(1024, 1024, Nitr);
        data.I0 = zeros(camera.Neta, camera.Nxi, target.broadSampleNum); % used to save the original focal images
        data.I = zeros(camera.Neta, camera.Nxi, target.broadSampleNum, Nitr); % used to save the focal images after each control iteration
        data.Ip = zeros(camera.Neta, camera.Nxi, target.broadSampleNum, 2*estimator.NumImgPair+1, Nitr);
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
        data.y0 = zeros(darkHole.pixelNum, estimator.NumImgPair);
        data.P0 = zeros(2, 2, darkHole.pixelNum);
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
        data.estimatedContrastErr = zeros(Nitr, 1);
        data.DMcommand = zeros(2 * DM.activeActNum, Nitr); % the DM control command for each iteration
        data.backgroundAverage = zeros(Nitr, 1); % the average contrast of background
        data.backgroundStd = zeros(Nitr, 1); % the std deviation of the backgroud
        data.probeContrast = zeros(Nitr, 1); % the probe contrast
        if strcmpi(estimator.type, 'ekf')% && ~estimator.EKFpairProbing
            if estimator.nonProbeImage
                data.y = zeros(darkHole.pixelNum, estimator.NumImg+1, Nitr);
            else
                data.y = zeros(darkHole.pixelNum, estimator.NumImg, Nitr); % the difference images
            end
        else
            if estimator.EKFpairProbing
                data.y = zeros(darkHole.pixelNum, estimator.NumImgPair, Nitr); % the difference images
            else
                data.y = zeros(darkHole.pixelNum, estimator.NumImg, Nitr); % the difference images
            end
        end
        
        switch lower(estimator.type)
            case {'perfect'}
            case {'kalman', 'batch'}
                data.P = zeros(2, 2, darkHole.pixelNum, Nitr);
                if estimator.EKFpairProbing
                    if strcmpi(estimator.whichDM, 'both')
                        data.uProbe = zeros(DM.activeActNum*2, estimator.NumImgPair, Nitr); % the probe shapes
                    else
                        data.uProbe = zeros(DM.activeActNum, estimator.NumImgPair, Nitr); % the probe shapes
                    end
                else
                    if strcmpi(estimator.whichDM, 'both')
                        data.uProbe = zeros(DM.activeActNum*2, estimator.NumImg, Nitr); % the probe shapes
                    else
                        data.uProbe = zeros(DM.activeActNum, estimator.NumImg, Nitr); % the probe shapes
                    end
                end
            case {'ekf', 'ukf'}
                if estimator.EKFincoherent
                    data.P = zeros(3, 3, darkHole.pixelNum, Nitr);
                else
                    data.P = zeros(2, 2, darkHole.pixelNum, Nitr);
                end
                if strcmpi(estimator.whichDM, 'both')
                    data.uProbe = zeros(DM.activeActNum*2, estimator.NumImg, Nitr); % the probe shapes
                else
                    data.uProbe = zeros(DM.activeActNum, estimator.NumImg, Nitr); % the probe shapes
                end
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
data.estimator = estimator;
if strcmpi(simOrLab, 'simulation')
    data.EfocalPerfect = zeros(darkHole.pixelNum, Nitr);
end