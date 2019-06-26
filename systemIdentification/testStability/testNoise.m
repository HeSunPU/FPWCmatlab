NumImg = 5000;
IseriesDark = zeros(darkHole.pixelNum, NumImg);
for itr = 1:NumImg
    disp(num2str(itr));
    I = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
    IseriesDark(:, itr) = I(darkHole.pixelIndex);
end
%%

Isimulation = int16(18 * ones(1, 30000));
Ipoisson = poissrnd(10, size(Isimulation));
Igaussian = normrnd(0, 4, size(Isimulation));
Isim = Ipoisson + double(Igaussian); 

IsimGaussian = normrnd(10, 4, size(Isimulation));