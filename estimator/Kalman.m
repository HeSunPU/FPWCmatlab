function [EfocalEst, IincoEst, data] = Kalman(u, image, darkHole, model, estimator, controller, data, kWavelength)
%% The pixel-wise Kalman filter estimator
% Developed by He Sun on Feb. 24, 2017
%
% EfocalEst - the estimated coherent focal plane electric field
% IincoEst - the estimated incoherent focal plane intensity
% u - the DM probing commands
% image - the images used for wavefront estimation
% darkHole - defines the parameters of the dark holes
% model - the control Jacobian and other model related variables
% controller - the properties of the controller
% estimator - the properties of estimator
%
%
% check number of inputs
if nargin > 8
    disp('Wrong number of input parameters!!');
elseif nargin < 8
    kWavelength = 0;
end
% assert we do want to use Kalman filter estimator
% assert(strcmpi(estimator.type, 'kalman'), 'Not using pixel-wise Kalman filter estimator now!');
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
        disp('We only have DM 1, 2 or both for probing!');
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
EfocalEst = zeros(darkHole.pixelNum, 1);
IincoEst = zeros(darkHole.pixelNum, 1);

% generate the control Jacobian and control command used according to
% the DM we use for control
if data.itr == 1
    command = data.DMcommand(:, 1);
else
    command = data.DMcommand(:, data.itr) - data.DMcommand(:, data.itr - 1);
end
switch controller.whichDM
    case '1'
        G = model.G1;
        command = command(1 : length(command)/2);
    case '2'
        G = model.G2;
        command = command(length(command)/2 + 1, length(command));
    case 'both'
        G = [model.G1, model.G2];
    otherwise
        disp('You can only use the first DM, second DM or both for wavefront control.');
        return;
end
% generate the covariance matrices
R = 2 * estimator.observationVarCoefficient * eye(estimator.NumImgPair);
Q = (sum(command.^2) + 0.3) * estimator.processVarCoefficient * eye(2);
% Q = estimator.processVarCoefficient * eye(2);
% Kalman filter for each pixel in the dark holes
for q = 1 : darkHole.pixelNum
    if kWavelength == 0
        if data.itr == 1
            xOld = [real(data.EfocalEst0(q)); imag(data.EfocalEst0(q))];
        else
            xOld = [real(data.EfocalEst(q, data.itr-1)); imag(data.EfocalEst(q, data.itr-1))];
        end
    else
        if data.itr == 1
            xOld = [real(data.EfocalEst0(q, kWavelength)); imag(data.EfocalEst0(q, kWavelength))];
        else
            xOld = [real(data.EfocalEst(q, kWavelength, data.itr-1)); imag(data.EfocalEst(q, kWavelength, data.itr-1))];
        end
    end
    y = Idiff(:, q);
    H = 4 * [real(probe(:, q)), imag(probe(:, q))];
    update = [real(G(q, :)); imag(G(q, :))] * command; % the linear update of the state
    % Kalman filter estimation officially starts here
    xPriori = xOld + update; % predict a priori state estimate
    if data.itr == 1
        Ppriori = estimator.stateStd0 * eye(2) + Q;
    else
        if kWavelength == 0
            Ppriori = data.P(1:2, 1:2, q, data.itr-1) + Q; % predict a priori estimate covariance
        else
            Ppriori = data.P(1:2, 1:2, q, kWavelength, data.itr-1) + Q; % predict a priori estimate covariance
        end
    end
    residual = y - H * xPriori; % compute the measurement residual
    S = H * Ppriori * H' + R; % residual covariance
    K = Ppriori * H' / S; % optimal Kalman gain
    xPosteriori = xPriori + K * residual; % update a posteriori state estimate
    Pposteriori = (eye(2) - K * H) * Ppriori; % update a posteriori estimate covariance
    % save the data
    EfocalEst(q) = xPosteriori(1) + 1i * xPosteriori(2);
    IincoEst(q) = InoPoke(q) - sum(xPosteriori.^2);
%         % try to use probing images in estimating the incoherent light 
%         IincoEstNoPoke = InoPoke(q) - sum(xPosteriori.^2);
%         IincoEstPoke = zeros(2 * estimator.NumImgPair, 1);
%         for k = 1 : estimator.NumImgPair
%             IpositivePoke2D = image(:, :, 2*k);
%             IpositivePoke = IpositivePoke2D(darkHole.pixelIndex); % the image with positive poking
%             InegativePoke2D = image(:, :, 2*k+1);
%             InegativePoke = InegativePoke2D(darkHole.pixelIndex); % the image with negative poking
%             IincoEstPoke(2*k-1) = IpositivePoke(q) - (xPosteriori(1) + real(probe(k, q)))^2 - (xPosteriori(2) + imag(probe(k, q)))^2;
%             IincoEstPoke(2*k) = InegativePoke(q) - (xPosteriori(1) - real(probe(k, q)))^2 - (xPosteriori(2) - imag(probe(k, q)))^2;
%         end
%         IincoEst(q) = mean([IincoEstNoPoke; IincoEstPoke]);
    if kWavelength == 0
        data.P(1:2, 1:2, q, data.itr) = Pposteriori;
    else
        data.P(1:2, 1:2, q, kWavelength, data.itr) = Pposteriori;
    end
end
end