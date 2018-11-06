function target = targetDrift(target)
%% The drift of the target over time
% Developed by He Sun on Nov. 6, 2018
% taraget - defines the properties of light source

if target.drift
    %% random walk of drift coefficients
    target.driftCoef = target.driftCoef + normrnd(0, target.driftStd, [target.NdriftMode, 1]);
    %% record the new set of coefficients
    target.driftCoefCollection = [target.driftCoefCollection, target.driftCoef];
end
end