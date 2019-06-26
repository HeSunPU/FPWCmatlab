%% "LDSlearning.m" - the main function to run focal plane wavefront control and estimation
% Developed by He Sun on Mar. 11th, modified on Nov. 9th
% You can run the code in two mode, offline/analytical learning using
% "systemIdentify.m" or online learning using "onlineLearning.m"

%% use matlab "parpool" function to update the model of different pixels
% simultaneously
parpool(16);

%% initialization of the model
G1Learned = model.G1;
G2Learned = model.G2;
Qlearned = zeros(2, 2, size(model.G1, 1));
Rlearned = zeros(2, 2, size(model.G1, 1));
index = 495;
n_data = 2000;
Nitr = 2;

%% start identifying the model of different pixels
parfor index = 1: size(model.G1, 1)
    
    %% prepare the data
    disp(['Now we are learning pixel ', num2str(index)]);
    G1 = [real(model.G1(index, :)); imag(model.G1(index, :))];
    G2 = [real(model.G2(index, :)); imag(model.G2(index, :))];
%     G = [G1, G2];
    G = [G1, G2];
    %% initialize the x0 and P0
    %u = [data.u1; data.u2];
    u = [data.u1(:, 1:n_data); data.u2(:, 1:n_data)];
    uProbe = [data.uProbe(:, :, 1:n_data); zeros(952, 2, n_data)];
    image0 = data.image0;
    H = zeros(numImgPair, 2);
    p = zeros(numImgPair, 2);
    for k = 1 : numImgPair
        p(k, 1) = G(1, :) * uProbe(:, k);
        p(k, 2) = G(2, :) * uProbe(:, k);
        H(k, 1) = 4 * p(k, 1);
        H(k, 2) = 4 * p(k, 2);
    end
    for k = 1 : numImgPair
        amp = 0.5 * (data.Iplus0(index, k) + data.Iminus0(index, k)) -  data.I0(index);
        amp = sqrt(amp);
        ampModel = sqrt(p(k, 1)^2 + p(k, 2)^2);
        H(k, :) = amp / ampModel * H(k, :);
    end
    Q = 3e-9 * eye(2);
    R = 3e-14 * eye(numImgPair);
    Hinv = pinv(H);
    x0 = Hinv * image0(index, :)';
    P0 = Hinv * R * Hinv';
    y = squeeze(data.image(index, :, 1:n_data));
    
%     %% offline learning
%     Nitr = 4;
%     [system, stateEst]= systemIdentify(u, y, G, H, Q, R, x0, P0, uProbe, Nitr);
%     G1Learned(index, :) = system.G(1, 1:size(model.G1, 2)) + 1i * system.G(2, 1:size(model.G1, 2));
% %     G2Learned(index, :) = system.G(1, size(model.G1, 2)+1:end) + 1i * system.G(2, size(model.G1, 2)+1:end);
%     Qlearned(:, :, index) = system.Q;
%     Rlearned(:, :, index) = system.R;
    
    %% online learning
    delta1 = 1e-1;
    delta2 = 1e-1;
    batchSize = 10; % how many observations for each updates
%     error_online = zeros(length(1 : batchSize : 3500), 1);
%     error_online_squared1 = zeros(length(1 : batchSize : 3500), 1);
% %     error_online_squared2 = zeros(length(1 : batchSize : 3500), 1);
%     validation_error_online = zeros(length(1 : batchSize : 3500), 1);
%     k = 1;
    for learningItr = 1 : batchSize : n_data
%         disp(k)
        u = [data.u1(:, learningItr : learningItr+batchSize-1); data.u2(:, learningItr : learningItr+batchSize-1)];
        uProbe = [data.uProbe(:, :, learningItr : learningItr+batchSize-1); zeros(952, 2, batchSize)];
        if(batchSize == 1)
            y = squeeze(data.image(index, :, learningItr : learningItr+batchSize-1))';
        else
            y = squeeze(data.image(index, :, learningItr : learningItr+batchSize-1));
        end
        [system, stateEst]= onlineLearning(u, y, G, Q, R, x0, P0, uProbe, Nitr, delta1, delta2);
        G1Learned(index, :) = system.G(1, 1:size(model.G1, 2)) + 1i * system.G(2, 1:size(model.G1, 2));
        G2Learned(index, :) = system.G(1, size(model.G1, 2)+1:end) + 1i * system.G(2, size(model.G1, 2)+1:end);
        Q = system.Q;
        R = system.R;
        G = system.G;
        x0 = stateEst.x(:, end);
        P0 = stateEst.P(:, :, end);
%         error_online(k) = norm(G1Learned(index, :) - modelTrue.G1(index, :), 2)^2/ norm(modelTrue.G1(index,:), 2)^2;
%         error_online_squared1(k) = norm(G1Learned(index, :)'*G1Learned(index, :) - modelTrue.G1(index, :)'*modelTrue.G1(index, :), 2)^2/ norm(modelTrue.G1(index,:)'*modelTrue.G1(index, :), 2)^2;
% %         error_online_squared2(k) = norm(uProbe(:, :, 1)'*(G1Learned(index, :)'*G1Learned(index, :) - modelTrue.G1(index, :)'*modelTrue.G1(index, :))*u, 2)^2/ norm(uProbe(:, :, 1)'*modelTrue.G1(index,:)'*modelTrue.G1(index, :)*u, 2)^2;
%         validation_error_online(k) = validationErr(dataValidation, G1Learned(index, :)) / norm(squeeze(dataValidation.deltay), 'fro')^2;
%         k = k + 1;
    end
    %%
%     validation_error_online500 = validation_error_online;
%     error_online_squared500 = error_online_squared1;
%     error_online500 = error_online;
end