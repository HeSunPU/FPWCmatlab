probe = zeros(1292, 2, 20);
for k = 1 : 20
    probe(:, :, k) = model.G1 * data.uProbe(:, :, k);
end

%%
figure, semilogy(abs(probe(:, 1, 1)))

%%
n = 6;
figure, semilogy(abs(rem(angle(probe(:, 1, n))-angle(probe(:, 2, n)), pi)) / pi * 180)

%%
figure, plot(data.uProbe(:, 1, 1))
%%
probe = model.G1 * u0;
figure, semilogy(abs(rem(angle(probe(:, 1))-angle(probe(:, 2)), pi)) / pi * 180)