function f = fitFocalLengthIFS(I, target, coronagraph, camera)
    options = optimset('Display', 'iter', 'PlotFcns', @optimplotfval, 'TolFun', 1e-4, 'TolX', 1e-4);
    fun = @(x)ImgErr(I, x, target, coronagraph, camera);
    x0 = 2.9;%1.244;
    f = fminsearch(fun, x0, options);
end
function out = ImgErr(I, focalLength, target, coronagraph, camera)
    SPshape = imresize(coronagraph.SPshape, [coronagraph.Nsp, coronagraph.Nsp], 'bicubic'); 
    EspOut = 1.0 * SPshape; % electric field leaving the shaped pupil mask
    Esim = Fourier(EspOut, focalLength, target.starWavelength, coronagraph.SPwidth, coronagraph.SPwidth,...
        focalLength/3.1214e+04, camera.Nxi, focalLength/3.1214e+04, camera.Neta); % electric field of the focal plane5.9296e+04
    Isim = abs(Esim).^2;
    Isim = Isim/max(max(Isim));
    out = sum(sum((Isim - I).^2));
end