function output = func_cost_for_BMC_inf_fit(sigma, infMeas)
%% the cost function for fitting influencing function
pitch = 300e-6;
dx = pitch / 13.02;
N = length(infMeas);
xs = ((-N/2 : N/2-1) + 1/2) * dx / pitch;
[XS, YS] = meshgrid(xs, xs);
inf0 = exp(-4*log(2)*((XS/(sigma)).^2 + (YS/(sigma)).^2));
output = sum(sum(abs(inf0 - infMeas).^2));
end