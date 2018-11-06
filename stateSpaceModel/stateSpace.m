function model = stateSpace(target, DM, coronagraph, camera, darkHole)
% the function that calculates the state space model (control Jacobian
% matrix) of the system
% Developed by He Sun on Feb. 16, 2017
%
% model - the output state space model
% DM - defines the DM model and parameters of devices
% coronagraph - defines the coronagraph type, shape and distances
% camera - defines the properties of camera, including pixel size, binning,
% noises and others
% darkHole - defines the dark hole region

%% Only consider the star in light source
target.star = 1;
target.planet = 0;
DM.noise = 0;
camera.noise = 0;
coronagraph.error = 0;

%% Compute the focal plane electric field given current voltage
[EnoPoke, ~, ~]= opticalModel(target, DM.DMperfect, coronagraph.coronagraph_perfect, camera, DM.DM1command, DM.DM2command);

%% Compute the control Jacobian matrix of DM1 - G1
disp('Calculating the control Jacobian of DM1 ...');
G1 = zeros(darkHole.pixelNum, DM.activeActNum);
% add small perturbation to each actuator to calculate Jacobian
if strcmpi(coronagraph.type, 'WFIRST')
    dVolt = 1.5;
else
    dVolt = 1/4;
end

parfor k = 1 : DM.activeActNum
    DM1commandPoke = DM.DM1command;
    DM1commandPoke(k) = DM1commandPoke(k) + dVolt;
    [Epoke, ~, ~] = opticalModel(target, DM.DMperfect, coronagraph.coronagraph_perfect, camera, DM1commandPoke, DM.DM2command);
    G1(:, k) = (1 / dVolt) * (Epoke(darkHole.pixelIndex) - EnoPoke(darkHole.pixelIndex));
end

%% Compute the control Jacobian matrix of DM2 - G2
disp('Calculating the control Jacobian of DM2 ...');
G2 = zeros(darkHole.pixelNum, DM.activeActNum);
% add small perturbation to each actuator to calculate Jacobian
dVolt = 1/4;
parfor k = 1 : DM.activeActNum
    DM2commandPoke = DM.DM2command;
    DM2commandPoke(k) = DM2commandPoke(k) + dVolt;
    [Epoke, ~, ~] = opticalModel(target, DM.DMperfect, coronagraph.coronagraph_perfect, camera, DM.DM1command, DM2commandPoke);
    G2(:, k) = (1 / dVolt) * (Epoke(darkHole.pixelIndex) - EnoPoke(darkHole.pixelIndex));
end

%%
model.G1 = G1;
model.G2 = G2;
disp('I am done!');
end