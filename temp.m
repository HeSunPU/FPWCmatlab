%%
probe_nonlinear = zeros(darkHole.pixelNum, 2);

[EnoPoke, ~, ~]= opticalModel(target, DM, coronagraph.coronagraph_perfect, camera, DM.DM1command, DM.DM2command);
for k = 1:2
    [EPoke1, ~, ~]= opticalModel(target, DM, coronagraph.coronagraph_perfect, camera, DM.DM1command+u(:, k), DM.DM2command);
    [EPoke2, ~, ~]= opticalModel(target, DM, coronagraph.coronagraph_perfect, camera, DM.DM1command-u(:, k), DM.DM2command);
    probe_nonlinear(:, k) = 0.5 * (EPoke1(darkHole.pixelIndex) - EPoke2(darkHole.pixelIndex));
end
%%
probe = model.G1 * u;

%%
figure, semilogy(abs(probe).^2)
figure, plot(abs(rem(angle(probe(:, 1)) - angle(probe(:, 2)), pi)/pi*180))

%%
figure, semilogy(abs(probe_nonlinear).^2)
figure, plot(abs(rem(angle(probe_nonlinear(:, 1)) - angle(probe_nonlinear(:, 2)), pi)/pi*180))

%%
I0 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
figure(5), imagesc(log10(abs(I0))), colorbar;
% caxis(cRange);
drawnow
