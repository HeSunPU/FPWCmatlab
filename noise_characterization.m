

% characterize the noise parameters
EfocalEst = data.EfocalPerfect;%data.EfocalEst;
EfocalEst0 = data.EfocalEst0;
uc = zeros(DM.activeActNum*2, Nitr);
uc(:, 1) = data.DMcommand(:, 1);
for k = 2 : Nitr
    uc(:, k) = data.DMcommand(:, k) - data.DMcommand(:, k-1);
end
dEfocalEst = zeros(darkHole.pixelNum, Nitr);
for k = 1 : Nitr
    if k == 1
        dEfocalEst(:, k) = EfocalEst(:, k) - EfocalEst0;
    else
        dEfocalEst(:, k) = EfocalEst(:, k) - EfocalEst(:, k-1);
    end
end
G = [model.G1, model.G2];
dEfocalEst_model = G * uc;

%%
uc_sum_abs = sum(abs(uc), 1);
uc_sum_cubic = sum(abs(uc).^3, 1);
uc_sum = sum(uc.^2, 1);
update_square = mean(abs(dEfocalEst_model).^2, 1);
cur_contrast = mean(abs(EfocalEst).^2, 1);
update_cubic = mean(abs(dEfocalEst_model).^3, 1);
process_noise = mean(abs(dEfocalEst - dEfocalEst_model).^2, 1);
process_noise_max = max(abs(dEfocalEst - dEfocalEst_model).^2, [], 1); 
process_noise_med = median(abs(dEfocalEst - dEfocalEst_model).^2, 1);
y = process_noise(:, 3:end)';
% H = [uc_sum(:, 3:end)', ones(Nitr-2, 1)];
% H = [uc_sum_cubic(:, 3:end)', ones(Nitr-2, 1)];
% H = [update_cubic(:, 5:end)', ones(Nitr-4, 1)];
H = [update_square(:, 3:end)', ones(Nitr-2, 1)];
x = (H'*H)^(-1) * H' * y;
% H2 = [uc_sum_cubic', uc_sum', uc_sum_abs', ones(80, 1)];
% x2 = (H2'*H2)^(-1) * H2' * y;

%%
H = [dE_pred(:, 5:end)', ones(Nitr-4, 1)];
y = process_noise(:, 5:end)';
x = (H'*H)^(-1) * H' * y;
%%
up = data.uProbe;
Idiff = data.y;
Idiff_pred = zeros(size(Idiff));
up_sum = squeeze(sum(up.^2, 1));
up_sum_cubic = squeeze(sum(abs(up).^3, 1));
probe = zeros(darkHole.pixelNum, estimator.NumImgPair, Nitr);
for k = 1 : Nitr
    probe(:, :, k) = model.G1 * up(:, :, k);
    for j = 1 : estimator.NumImgPair
        Idiff_pred(:, j, k) = 4 * (real(probe(:, j, k)) .* real(EfocalEst(:, k)) + ...
            imag(probe(:, j, k)) .* imag(EfocalEst(:, k)));
    end
end
observ_noise = squeeze(mean(abs(Idiff - Idiff_pred).^2, 1));

%%
up = data.uProbe;
% Ip = data.y;
Ip_pred = zeros(size(Ip));
up_sum = squeeze(sum(up.^2, 1));
probe = zeros(darkHole.pixelNum, estimator.NumImg, Nitr);
for k = 1 : Nitr
    probe(:, :, k) = model.G1 * up(:, 1:estimator.NumImg, k);
    for j = 1 : estimator.NumImg
        Ip_pred(:, j, k) = abs(EfocalEst(:, k) + probe(:, j, k)).^2;
    end
end
observ_noise = squeeze(mean(abs(Ip(:, 1:estimator.NumImg, :) - Ip_pred(:, 1:estimator.NumImg, :)).^2, 1));
% observ_pred = squeeze(mean(abs(Ip_pred).^2, 1));
% observ_pred = squeeze(mean(abs(Ip).^2, 1));
observ_pred = squeeze(mean(Ip_pred, 1)).^2;
prob_mag = squeeze(mean(abs(probe).^2, 1));
observ_noprobe = mean(abs(EfocalEst).^2, 1);
%%
observ_noise = [observ_noise(1, 1:end), observ_noise(2, 1:end)];
observ_pred = [observ_pred(1, 1:end), observ_pred(2, 1:end)];
prob_mag = [prob_mag(1, 1:end), prob_mag(2, 1:end)];
up_sum = [up_sum(1, 1:end), up_sum(2, 1:end)];
%%
observ_noise = observ_noise(2, 1:end);
observ_pred = observ_pred(2, 1:end);
prob_mag = prob_mag(2, 1:end);
up_sum = up_sum(2, 1:end);


%%
% y = observ_noise';
% H = [up_sum', ones(80, 1)];
% H = [observ_pred', ones(80, 1)];
% H = [(prob_mag.^2)', ones(80, 1)];
y = observ_noise;
% H = [(sqrt(observ_pred).*prob_mag), ones(Nitr, 1)];
% H = [observ_noprobe' .* up_sum, ones(Nitr, 1)];
% H = [observ_noprobe' .* prob_mag.^(3/2), ones(Nitr, 1)];
% H = [observ_noprobe' .* up_sum_cubic, ones(Nitr, 1)];
% H = [observ_pred, ones(Nitr, 1)];
H = [prob_mag.^2 + observ_noprobe'.^2, ones(Nitr, 1)];


%%
y = log10(abs(observ_noise-5e-17));
H = [log10(observ_pred), ones(Nitr, 1)];
%%
x = (H'*H)^(-1) * H' * y;