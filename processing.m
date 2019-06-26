probe_old = zeros(darkHole.pixelNum, Nitr);

for k = 1 : 15
    probe_old(:, k) = model.G1*squeeze(data.uProbe(:, :, k));
end
probe_old_angle = atan2(imag(probe_old), real(probe_old));
probe_old_amp = abs(probe_old);

%%
probe_opt = zeros(darkHole.pixelNum, Nitr);
for k = 1 : 15
    probe_opt(:, k) = model.G1*squeeze(data.uProbe(1:952, :, k));% + model.G2*squeeze(data.uProbe(953:end, :, k));
end
probe_opt = data.EfocalEst + probe_opt;
probe_opt_angle = atan2(imag(probe_opt), real(probe_opt));
probe_opt_amp = abs(probe_opt);


%%
probe_opt = zeros(darkHole.pixelNum, Nitr);
for k = 1 : 15
    probe_opt(:, k) = model.G1*squeeze(data.uProbe(1:952, :, k));% + model.G2*squeeze(data.uProbe(953:end, :, k));
end
probe_opt_angle = atan2(imag(probe_opt), real(probe_opt));
probe_opt_amp = abs(probe_opt);
%%
for k = 1 : 14
% figure(1), plot(probe_old_angle(:, k+1)/pi*180 -  probe_old_angle(:, k)/pi*180)
figure(101), plot(mod(abs(probe_old_angle(:, k+1)/pi*180 -  probe_old_angle(:, k)/pi*180), 180))
drawnow
pause(1)
end

%%
for k = 1 : 14
% figure(1), plot(probe_old_angle(:, k+1)/pi*180 -  probe_old_angle(:, k)/pi*180)
figure(102), plot(mod(abs(probe_opt_angle(:, k+1)/pi*180 -  probe_opt_angle(:, k)/pi*180), 180))
drawnow
pause(1)
end

%%
figure(103), semilogy(1:15, mean(probe_old_amp, 1), 'r-', ...
    1:15, mean(probe_opt_amp, 1), 'k-')

%%
for k = 1 : 15
    figure(104), semilogy(1:darkHole.pixelNum, probe_old_amp(:, k), 'r-', ...
        1:darkHole.pixelNum, probe_opt_amp(:, k), 'k-')
    drawnow
    pause(1)
end
%%
Iprobe_opt = zeros(camera.Neta, camera.Nxi);
Iprobe_old = zeros(camera.Neta, camera.Nxi);
for k = 1 : 15
    Iprobe_old(darkHole.pixelIndex) = probe_old_amp(:, k);
    Iprobe_opt(darkHole.pixelIndex) = probe_opt_amp(:, k);
    
    figure(105), imagesc(log(abs(Iprobe_old))), colorbar
    figure(106), imagesc(log(abs(Iprobe_opt))), colorbar
    drawnow
    pause(1)
end

%%
Efocal_est = zeros(camera.Neta, camera.Nxi);
Efocal_perfect = zeros(camera.Neta, camera.Nxi);
for k = 1 : 15
    Efocal_est(darkHole.pixelIndex) = data.EfocalEst(:, k);
    Efocal_perfect(darkHole.pixelIndex) = EfocalPerfect(:, k);
    figure(107), imagesc(log(abs(Efocal_est).^2)), colorbar
    caxis([-24, -12]);
    figure(108), imagesc(log(abs(Efocal_perfect).^2)), colorbar
    caxis([-24, -12]);
    drawnow
    pause(1)
end

%%
E_est = data.EfocalEst;
E_true = EfocalPerfect;
for itrID = 1 : 10
pixID = 428;

P_est = squeeze(data.P(:, :, pixID, :));
x = [real(E_est(pixID, itrID)), imag(E_est(pixID, itrID))];
x_true = [real(E_true(pixID, itrID)), imag(E_true(pixID, itrID))];
P = P_est(1:2, 1:2, itrID);
[X, Y]= error_ellip(x, P, 100);

figure(110)
plot([0, x(1)], [0, x(2)], 'r-', ...
    x(1), x(2), 'ro', ...
    [0, x_true(1)], [0, x_true(2)], 'b-', ...
    x_true(1), x_true(2), 'bx', ...
    X, Y, 'r--')
title(num2str(itrID))
xlim([-5*abs(x_true(1)), 5*abs(x_true(1))]);
ylim([-5*abs(x_true(2)), 5*abs(x_true(2))]);
drawnow
pause(2)
end

%%
figure, plot(1:2092, abs(E_true(:, 3)), 'b-', ...
    1:2092, abs(E_est(:, 3)), 'r-')

%%
u1c = zeros(952, 15);
u2c = zeros(952, 15);
for k = 1 : 15
    if k == 1
        u1c(:, k) = data.DMcommand(1:952, 1);
        u2c(:, k) = data.DMcommand(953:end, 1);
    else
        u1c(:, k) = data.DMcommand(1:952, k) - data.DMcommand(1:952, k-1);
        u2c(:, k) = data.DMcommand(953:end, k) - data.DMcommand(953:end, k-1);
    end
end

Ec = zeros(darkHole.pixelNum, 15);
Ec_nonlinear = zeros(darkHole.pixelNum, 15);
Ep = zeros(darkHole.pixelNum, 15);

for k = 1 : 15
    Ec(:, k) = model.G1 * u1c(:, k) + model.G2 * u2c(:, k);
    if k == 1
        [Enp, ~, ~] = opticalModel(target, DM.DMperfect, coronagraph, camera, zeros(952, 1), zeros(952, 1));
    else
        [Enp, ~, ~] = opticalModel(target, DM.DMperfect, coronagraph, camera, data.DMcommand(1:952, k-1), data.DMcommand(953:end, k-1));
    end
    [Ep1, ~, ~] = opticalModel(target, DM.DMperfect, coronagraph, camera, data.DMcommand(1:952, k), data.DMcommand(953:end, k));
    Ec_nonlinear(:, k) = Ep1(darkHole.pixelIndex) - Enp(darkHole.pixelIndex);
    Ep(:, k) = model.G1 * squeeze(data.uProbe(:, :, k));
end

%%
for k = 1 : 14
figure(111), plot(1:darkHole.pixelNum, real(E_true(:, k+1)-E_true(:, k)), 'b-', ...
    1:darkHole.pixelNum, real(Ec(:, k+1)), 'r-', ...
    1:darkHole.pixelNum, real(E_true(:, k+1)-E_true(:, k)-Ec(:, k+1)), 'k-')
title(num2str(k))
drawnow
figure(112), plot(u1c(:, k), 'b-')
title(num2str(k))
drawnow
figure(113), plot(u2c(:, k), 'b-')
title(num2str(k))
drawnow
pause(2)

end

%%
var_list = zeros(14, 1);
usquared = zeros(14, 1);
for k = 1 : 14
    temp1 = sum(abs(E_true(:, k+1)-E_true(:, k)-Ec(:, k+1)).^2);%max(abs(E_true(:, k+1)-E_true(:, k)-Ec(:, k+1)).^2);%
    temp2 = sum(u1c(:, k+1).^2+u2c(:, k+1).^2);
    disp([num2str(k+1), ', ', num2str(temp1), ', ', num2str(temp2), ', ', num2str(temp1/temp2)]);
    var_list(k) = temp1/darkHole.pixelNum;%temp1;%
    usquared(k) = temp2;
end
y = var_list;
H = [usquared, ones(14, 1)];
x = (H'*H)^(-1)*H'*y;

%%
var_list2 = zeros(15, 1);
upsquared = zeros(15, 1);
for k = 1 : 15
    temp1 = sum((squeeze(data.y(:, :, k))-abs(E_true(:, k)+ Ep(:, k)).^2).^2);%max(abs(E_true(:, k+1)-E_true(:, k)-Ec(:, k+1)).^2);%
    temp2 = sum(data.uProbe(:, :, k).^2);
    disp([num2str(k), ', ', num2str(temp1), ', ', num2str(temp2), ', ', num2str(temp1/temp2)]);
    var_list2(k) = temp1/darkHole.pixelNum;%temp1;%
    upsquared(k) = temp2;
end
y2 = var_list2;
H2 = [upsquared, ones(15, 1)];
x2 = (H2'*H2)^(-1)*H2'*y2;

%%
temp = py.numpy.load('params_EKF2.npy');
temp2 = temp.tolist;
model2.G1 = double(temp2{1}) + 1i * double(temp2{2});
model2.G2 = double(temp2{3}) + 1i * double(temp2{4});


%%
figure, plot(1:14, H*x, 'b-', ...
    1:14, y, 'r-', ...
    1:14, y-H*x, 'k-')

%%
q_pred = zeros(14, 1);
q_pred2 = zeros(14, 1);
for k = 1 : 14
%     q_pred(k) = (sum(u1c(:, k+1).^2 +u2c(:, k+1).^2) + 0.3) * estimator.processVarCoefficient;
    q_pred(k) = sum(u1c(:, k+1).^2 +u2c(:, k+1).^2) * estimator.processVarCoefficient + estimator.processVarCoefficient2;
    q_pred2(k) = sum(u1c(:, k+1).^2 +u2c(:, k+1).^2) * x(1);% + x(2);
end
figure, semilogy(1:14, q_pred, 'r', 1:14, y, 'b', 1:14, q_pred2, 'r--')

%%
r_pred = zeros(15, 1);
r_pred2 = zeros(15, 1);
for k = 1 : 15
%     q_pred(k) = (sum(u1c(:, k+1).^2 +u2c(:, k+1).^2) + 0.3) * estimator.processVarCoefficient;
    r_pred(k) = sum(data.uProbe(:, :, k).^2) * estimator.observationVarCoefficient2 + estimator.observationVarCoefficient;
    r_pred2(k) = sum(data.uProbe(:, :, k).^4) * x2(1) + 1e-14;% + x2(2);
end
figure, semilogy(1:15, r_pred, 'r', 1:15, y2, 'b', 1:15, r_pred2, 'r--')

%%
for k = 1 : 15
    figure(120), plot(1:darkHole.pixelNum, real(data.EfocalEst(:, k)), 'r-', ...
        1:darkHole.pixelNum, real(EfocalPerfect(:, k)), 'b-', ...
        1:darkHole.pixelNum, real(data.EfocalEst(:, k)-EfocalPerfect(:, k)), 'k-')
    title(num2str(k))
    drawnow
    pause(2)
end