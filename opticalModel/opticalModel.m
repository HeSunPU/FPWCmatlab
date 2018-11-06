function [EfocalStar, EfocalPlanet, Ifocal] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command)
%% The optical model of the HCIL testbed
% Developed by He Sun on Feb. 7, 2017
%
% Efocal - the complex focal plane electric field 
% target - defines the properties of light source
% DM - defines the DM model and parameters of devices
% coronagraph - defines the coronagraph type, shape and distances
% camera - defines the properties of camera, including pixel size, binning,
% noises and others

%% Calculate the DM surface shape
% convert the 1D command vector to 2D command matrix
assert(length(DM1command) == DM.activeActNum, 'The length of DM1 voltage command is wrong!');
assert(length(DM2command) == DM.activeActNum, 'The length of DM1 voltage command is wrong!');
DM1command2D = zeros(DM.Nact, DM.Nact);
DM2command2D = zeros(DM.Nact, DM.Nact);
DM1command2D(DM.activeActIndex) = DM1command;
DM2command2D(DM.activeActIndex) = DM2command;
% convert the voltage to height
DM1shape = surfaceShape(DM, DM1command2D, '1'); % shape(heights) of the first DM in meters
DM2shape = surfaceShape(DM, DM2command2D, '2'); % shape(heights) of the first DM in meters


%% compute the current drift
if target.drift
    surfDrift = zeros(size(target.driftModes, 1), size(target.driftModes, 2));
    for kMode = 1 : target.NdriftMode
        surfDrift = surfDrift + target.driftCoef(kMode) * target.driftModes(:, :, kMode);
    end
end
%% Propagation of star source
switch target.star
    case 1
        % the wavelength of the star
        lambda = target.starWavelength;
        % the incident field from star
        if target.drift
            EinStar = target.EinStar .* exp(1i * 2 * pi * surfDrift / lambda);
        else
            EinStar = target.EinStar;
        end
        % the focal plane star field
        if strcmpi(coronagraph.type, 'SPLC')
            EfocalStar = propagationSPLC(EinStar, lambda, DM1shape, DM2shape, DM, coronagraph, camera); 
        elseif strcmpi(coronagraph.type, 'SPC')
            EfocalStar = propagationSPC(EinStar, lambda, DM1shape, DM2shape, DM, coronagraph, camera);   
        end
    case 0
        EfocalStar = zeros(camera.Neta, camera.Nxi); % the focal plane star field
    otherwise
        disp('The target.star should be either 1 or 0!');
        return;
end

%% Propagation of planet source
switch target.planet
    case 1
        % the wavelength of the planet
        lambda = target.planetWavelength;
        % the incident field from planet
        if target.drift
            EinPlanet = target.EinPlanet .* exp(1i * 2 * pi * surfDrift / lambda);
        else
            EinPlanet = target.EinPlanet;
        end
        % the focal plane planet field
        if strcmpi(coronagraph.type, 'SPLC')
            EfocalPlanet = propagationSPLC(EinPlanet, lambda, DM1shape, DM2shape, DM, coronagraph, camera);
        elseif strcmpi(coronagraph.type, 'SPC')
            EfocalPlanet = propagationSPC(EinPlanet, lambda, DM1shape, DM2shape, DM, coronagraph, camera);
        end
    case 0
        EfocalPlanet = zeros(camera.Neta, camera.Nxi); % the focal plane planet field
    otherwise
        disp('The target.planet should be either 1 or 0!');
        return;
end
%% Normalize the field
EfocalStar = EfocalStar / sqrt(target.normalization);
EfocalPlanet = EfocalPlanet / sqrt(target.normalization);
Ifocal = abs(EfocalStar).^2 + abs(EfocalPlanet).^2;
end

function out = propagationSPC(Ein, lambda, DM1shape, DM2shape, DM, coronagraph, camera)
% propagation in the optical system of a specific light source
mirrorFactor = 2; % the mirror double the height influence
% propagate from incident field to DM1
Ndm = size(DM1shape, 1); % pixel number in one direction on DM
if (coronagraph.error)
    DM1error = imresize(coronagraph.DM1error, size(DM1shape));
    DM1shape = DM1shape + DM1error;
end
DM1phase = exp(2 * 1i * pi * mirrorFactor * DM1shape / lambda); % phase perturbations on DM1, pay attention to the negative sign here
EDM1 = Ein .* DM1phase; % electric field after DM1 phase perturbation
EDM1Pad = padarray(EDM1, [floor(Ndm/2), floor(Ndm/2)]); % pad the field  with zeros

% propagate from DM1 to DM2
NdmExpand = size(EDM1Pad, 1); % pixel number of augmented field
L = DM.widthDM * NdmExpand / Ndm; % width of augmented field
if (coronagraph.error)
    DM2error = imresize(coronagraph.DM2error, size(DM2shape));
    DM2shape = DM2shape + DM2error;
end
DM2shapePad = padarray(DM2shape, [floor(Ndm/2), floor(Ndm/2)]);
DM2phasePad = exp(2 * 1i * pi * mirrorFactor * DM2shapePad / lambda); % pay attention the negative sign here
EDM2Pad = Fresnel(EDM1Pad, lambda, L, DM.zDM1toDM2) .* DM2phasePad; % electric field after DM2 phase perturbation

% propagation in coronagraphic imaging system
EspIn = Fresnel(EDM2Pad,lambda, L, coronagraph.zDM2toSP); % electric field entering the shaped pupil
Nsp = coronagraph.Nsp; % resize the shaped pupil matrix to make the pixel size same as entering field matrix
SPshape = imresize(coronagraph.SPshape, [Nsp, Nsp], 'bicubic'); 
% crop the entering electric field
if mod(Nsp, 2) == 0
    EspInCrop = EspIn(round(NdmExpand/2) + 1 - Nsp/2 : round(NdmExpand/2) + Nsp/2, round(NdmExpand/2) + 1 - Nsp/2 : round(NdmExpand/2) + Nsp/2);
elseif mod(Nsp, 2) == 1
    EspInCrop = EspIn(round(NdmExpand/2) - floor(Nsp/2) : round(NdmExpand/2) + floor(Nsp/2), round(NdmExpand/2) - floor(Nsp/2) : round(NdmExpand/2) + floor(Nsp/2));
else
    disp('Wierd calculation of the shape pupil mesh!');
    return;
end
EspOut = EspInCrop .* SPshape; % electric field leaving the shaped pupil mask
if (coronagraph.error)
    SPerror = imresize(coronagraph.SPerror, size(EspOut));
    EspOut = EspOut .* exp(2 * 1i * pi * mirrorFactor * SPerror / lambda);
end
out = Fourier(EspOut, coronagraph.focalLength, lambda, coronagraph.SPwidth, coronagraph.SPwidth,...
    camera.pitch * camera.binXi, camera.Nxi, camera.pitch * camera.binEta, camera.Neta); % electric field of the focal plane
end

function out = propagationSPLC(Ein, lambda, DM1shape, DM2shape, DM, coronagraph, camera)
%%
mirrorFactor = 2;

if (coronagraph.error)
    DM1error = imresize(coronagraph.DM1error, size(DM1shape));
    DM1shape = DM1shape + DM1error;
    DM2error = imresize(coronagraph.DM2error, size(DM2shape));
    DM2shape = DM2shape + DM2error;
end


Ndm = size(DM1shape, 1);
% DM1phase = exp(2 * 1i * pi * mirrorFactor * DM1shape / lambda); % phase perturbations on DM1, pay attention to the negative sign here
% DM1phasePad = padarray(DM1phase, [floor(Ndm/2), floor(Ndm/2)]); % pad the field  with zeros
DM1phasePad = exp(2 * 1i * pi * mirrorFactor * padarray(DM1shape, [floor(Ndm), floor(Ndm)]) / lambda); % phase perturbations on DM1, pay attention to the negative sign here



NpupilPad = size(DM1phasePad, 1);
pupilPad = padarray(Ein.*coronagraph.apertureMask, floor((NpupilPad-coronagraph.Naperture)/2)*[1, 1]);
EDM1Pad = pupilPad.*DM1phasePad;

% propagation from DM1 to DM2
% DM2phase = exp(2 * 1i * pi * mirrorFactor * DM2shape / lambda); % phase perturbations on DM1, pay attention to the negative sign here
% DM2phasePad = padarray(DM2phase, [floor(Ndm/2), floor(Ndm/2)]); % pad the field  with zeros
DM2phasePad = exp(2 * 1i * pi * mirrorFactor * padarray(DM2shape, [floor(Ndm), floor(Ndm)]) / lambda); % phase perturbations on DM1, pay attention to the negative sign here
L = DM.widthDM * NpupilPad / DM.DMmesh(1);
EDM2Pad = Fresnel(EDM1Pad, lambda, L, DM.zDM1toDM2);
EDM2Pad = EDM2Pad .* DM2phasePad;

% propagate the field back from DM2 to DM1
pupilPad2 = Fresnel(EDM2Pad, lambda, L, -DM.zDM1toDM2);

% apply shape pupil to the field
% SPerror = imresize(coronagraph.SPerror, coronagraph.Nsp*[1, 1], 'bicubic');
% EspIn = exp(2 * 1i * pi *  SPerror / lambda) .* pupilPad2(NpupilPad/2+1-coronagraph.Nsp/2: NpupilPad/2+coronagraph.Nsp/2, NpupilPad/2+1-coronagraph.Nsp/2: NpupilPad/2+coronagraph.Nsp/2);
EspIn = pupilPad2(NpupilPad/2+1-coronagraph.Nsp/2: NpupilPad/2+coronagraph.Nsp/2, NpupilPad/2+1-coronagraph.Nsp/2: NpupilPad/2+coronagraph.Nsp/2);
EspOut = coronagraph.SPshape .* EspIn;

% Fourier transform to Fourier plane
Ef = Fourier(EspOut, coronagraph.focalLength, lambda, coronagraph.SPwidth, coronagraph.SPwidth,...
    coronagraph.FPMpitch, coronagraph.Nfpm, coronagraph.FPMpitch, coronagraph.Nfpm); % electric field of the focal plane
Ef2 = coronagraph.FPmask .* Ef;

% propagate from the first focal plane to Lyot plane
ElyotIn = Fourier(Ef2, coronagraph.focalLength, lambda, coronagraph.FPMpitch*coronagraph.Nfpm, coronagraph.FPMpitch*coronagraph.Nfpm,...
    coronagraph.lyotWidth/coronagraph.Nlyot, coronagraph.Nlyot, coronagraph.lyotWidth/coronagraph.Nlyot, coronagraph.Nlyot);
ElyotOut = coronagraph.LyotStop .* ElyotIn;


% Fourier transform from Lyot plane back to focal plane
out = Fourier(ElyotOut, coronagraph.focalLength, lambda, coronagraph.lyotWidth, coronagraph.lyotWidth,...
    camera.pitch*camera.binXi, camera.Nxi, camera.pitch*camera.binEta, camera.Neta);
end
