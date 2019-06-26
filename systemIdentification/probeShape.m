function surf = probeShape(target, coronagraph, estimator, XS, YS, offset, probeContrast)
% Calculate the probe shape
%
mx = (estimator.probeArea(2) - estimator.probeArea(1)) / coronagraph.SPwidth;
my = (estimator.probeArea(4) - estimator.probeArea(3)) / coronagraph.SPwidth; % frequency of the sinc wave, depending on the width of dark hole regions
wx = (estimator.probeArea(2) + estimator.probeArea(1)) / 2;
wy = (estimator.probeArea(4) + estimator.probeArea(3)) / 2; % frequency of the cosine wave, depending on the location of dark hole regions
SincAmp = target.starWavelength * sqrt(probeContrast) * sqrt(2 * pi); % amplitude of probe shape in meters;
surf = SincAmp * sinc(mx * (XS)) .* sinc(my * (YS)) .* ... 
    cos(2 * pi * wx / coronagraph.SPwidth * XS + offset) .* cos(2 * pi * wy / coronagraph.SPwidth * YS);
surf = pi * surf; % A.J. added an extra factor in the old code, seems like some black art.
end