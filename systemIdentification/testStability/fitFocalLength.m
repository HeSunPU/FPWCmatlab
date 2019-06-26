function f = fitFocalLength(I, target, coronagraph, camera)
    options = optimset('Display', 'iter', 'PlotFcns', @optimplotfval, 'TolFun', 1e-4, 'TolX', 1e-4);
    fun = @(x)ImgErr(I, x, target, coronagraph, camera);
    x0 = 1.244;
    f = fminsearch(fun, x0, options);
end
function out = ImgErr(I, focalLength, target, coronagraph, camera)
    SPshape = imresize(coronagraph.SPshape, [coronagraph.Nsp, coronagraph.Nsp], 'bicubic'); 
    EspOut = 1.0 * SPshape; % electric field leaving the shaped pupil mask
    Esim = Fourier(EspOut, focalLength, target.starWavelength, coronagraph.SPwidth, coronagraph.SPwidth,...
        camera.pitch * camera.binXi, camera.Nxi, camera.pitch * camera.binEta, camera.Neta); % electric field of the focal plane
    Isim = abs(Esim).^2;
    Isim = Isim/max(max(Isim));
    out = sum(sum((Isim - I).^2));
end