contrastEst = 8.5056e-05;
% u1p_values = py.numpy.array(min(sqrt(contrastEst * 1e8), 0.5) *rand(estimator.NumImgPair, DM.activeActNum));
u1p_values = py.numpy.array(u');

temp = py.sensing.optimal_probe11(estimator.sensor, u1p_values, 1, contrastEst, ...
                        1e-3, 1, 500);
u = double(temp);
u = u';

%%
probe = model.G1 * u;

%%
figure, semilogy(abs(probe).^2)
figure, plot(abs(rem(angle(probe(:, 1)) - angle(probe(:, 2)), pi)/pi*180))

%%
figure, plot(u)