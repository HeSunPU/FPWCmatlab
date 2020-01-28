function model = stateSpaceBroad(target, DM, coronagraph, camera, darkHole, bandwidth, weight)
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

%%
target_help = target;
model0 = stateSpace(target_help, DM, coronagraph, camera, darkHole);
%%
target_help.starWavelength = target.starWavelength * (1-0.5*bandwidth);
model1 = stateSpace(target_help, DM, coronagraph, camera, darkHole);
%%
target_help.starWavelength = target.starWavelength * (1+0.5*bandwidth);
model2 = stateSpace(target_help, DM, coronagraph, camera, darkHole);
%%
model.G1 = (weight(1)*model1.G1 + weight(2)*model0.G1 + weight(3)*model2.G1) / sum(weight);
model.G2 = (weight(1)*model1.G2 + weight(2)*model0.G2 + weight(3)*model2.G2) / sum(weight);
disp('I am done!');
end