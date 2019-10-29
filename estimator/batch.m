 function [EfocalEst, IincoEst, data] = batch(u, image, darkHole, model, estimator, data, kWavelength)
%%
% The pair-wise batch process estimator
% Developed by He Sun on Feb. 17, 2017
%
% EfocalEst - the estimated coherent focal plane electric field
% IincoEst - the estimated incoherent focal plane intensity
% u - the DM probing commands
% image - the images used for wavefront estimation
% darkHole - defines the parameters of the dark holes
% model - the control Jacobian and other model related variables
% estimator - the properties of estimator
%
% check number of inputs
if nargin > 7
    disp('Wrong number of input parameters!!');
elseif nargin < 7
    kWavelength = 0;
end
% assert we do want to use batch process estimator
% assert(strcmpi(estimator.type, 'batch'), 'Not using batch process estimator now!');
% assert we give correct probimg command input
assert(size(u, 2) == estimator.NumImgPair, 'Not giving correct DM probing commands!');
% assert we give correct number of images
assert(size(image, 3) == 1 + 2 * estimator.NumImgPair, 'Not giving correct number of images!');
% Extract the no poke image
InoPoke2D = image(:, :, 1);
InoPoke = InoPoke2D(darkHole.pixelIndex);
% Compute the pair-wise difference images and measured amplitude of probe
Idiff = zeros(estimator.NumImgPair, darkHole.pixelNum);
amplitude = zeros(estimator.NumImgPair, darkHole.pixelNum);
for k = 1 : estimator.NumImgPair 
    IpositivePoke2D = image(:, :, 2*k);
    IpositivePoke = IpositivePoke2D(darkHole.pixelIndex); % the image with positive poking
    InegativePoke2D = image(:, :, 2*k+1);
    InegativePoke = InegativePoke2D(darkHole.pixelIndex); % the image with negative poking
    Idiff(k, :) = IpositivePoke - InegativePoke; % difference images
    amplitudeSquare = (IpositivePoke + InegativePoke) /2 - InoPoke;
    amplitudeSquare(amplitudeSquare<0) = 0;
    amplitude(k, :) = sqrt(amplitudeSquare);
end
% Compuate the influence caused by probing
switch estimator.whichDM
    case '1'
        G = model.G1;
    case '2'
        G = model.G2;
    otherwise
        disp('We only have DM 1 or 2 for probing!');
        return;
end
probe = zeros(estimator.NumImgPair, darkHole.pixelNum);
for k = 1 : estimator.NumImgPair
    probe(k, :) = transpose(G * u(:, k));
end
% combine the phase information from model and amplitude information from
% measurement
if estimator.measuredAmp
    phase = atan2(imag(probe), real(probe));
    probe = amplitude .* (cos(phase) + 1i * sin(phase));
end
% conduct least square regression for each pixel
EfocalEst = zeros(darkHole.pixelNum, 1);
IincoEst = zeros(darkHole.pixelNum, 1);
if data.itr >= 0
%     R = 2 * data.backgroundStd(data.itr)^2 * eye(estimator.NumImgPair);
% else
    R = 2 * 3e-13 * eye(estimator.NumImgPair);
end
%%
for q = 1 : darkHole.pixelNum
    %%
    y = Idiff(:, q);
    H = 4 * [real(probe(:, q)), imag(probe(:, q))];
    Hinv = pinv(H);
    x = Hinv * y;
    EfocalEst(q) = x(1) + 1i * x(2);
    IincoEst(q) = InoPoke(q) - sum(x.^2);
%     % try to use probing images in estimating the incoherent light 
%     IincoEstNoPoke = InoPoke(q) - sum(x.^2);
%     IincoEstPoke = zeros(estimator.NumImgPair, 1);
%     for k = 1 : estimator.NumImgPair
%         IpositivePoke2D = image(:, :, 2*k);
%         IpositivePoke = IpositivePoke2D(darkHole.pixelIndex); % the image with positive poking
%         InegativePoke2D = image(:, :, 2*k+1);
%         InegativePoke = InegativePoke2D(darkHole.pixelIndex); % the image with negative poking
%         IincoEstPoke(k) = 0.5 * (IpositivePoke(q) + InegativePoke(q)) - sum(x.^2) - (abs(probe(k, q)))^2;
%     end
%     IincoEst(q) = sum([IincoEstNoPoke; 2 * IincoEstPoke])/(2*estimator.NumImgPair+1);
    
    if data.itr > 0
        if kWavelength == 0
            data.P(1:2,1:2,q,data.itr) = Hinv * R * Hinv';
            data.y(:, :, data.itr) = Idiff';
        else
            data.P(1:2, 1:2, q, kWavelength, data.itr) = Hinv * R * Hinv';
            data.yBroadband(:, :, kWavelength, data.itr) = Idiff';
        end
    else
        data.P0(1:2, 1:2, q) = Hinv * R * Hinv';
        data.y0 = Idiff';
    end
end
end