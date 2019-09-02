function datacube = extractCube(broadband_image)
    fitswrite(broadband_image, 'C:\Lab\FPWCmatlab\IFS\data\broadband_image.fits')
    fid = fopen('C:\Lab\FPWCmatlab\IFS\flags\newimgflag.txt', 'wt');
    fclose(fid);
    pause(1);
    if exist('C:\Lab\FPWCmatlab\IFS\flags\crispyflag.txt')==2
        while exist('C:\Lab\FPWCmatlab\IFS\flags\newcubeflag.txt')==0
            pause(1);
        end
        datacube = fitsread('C:\Lab\FPWCmatlab\IFS\data\cube.fits');
        pause(1);
        delete('C:\Lab\FPWCmatlab\IFS\flags\newcubeflag.txt');
        delete('C:\Lab\FPWCmatlab\IFS\data\cube.fits');
%         delete('C:\Lab\FPWCmatlab\IFS\data\broadband_image.fits')
    else
        disp('CRISPY is not running!!! Please initialize CRISPY first while using IFS!!!')
    end

end