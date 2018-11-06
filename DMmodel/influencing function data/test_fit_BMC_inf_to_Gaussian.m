% Fit the BMC kilo-C DM influence function to a Gaussian

% close all; clear all; clc;
% 
% cd ~/Dropbox/HiCAT_Princeton/FPWC/maps
%%
info = fitsinfo('inf_func_mean_BMC_HiCAT_13Nov2013.fits');
infMeas = fitsread('inf_func_mean_BMC_HiCAT_13Nov2013.fits'); % 4 actuator lengths across 52 pixels
N = length(infMeas);
%%
% dirhome = '~/Dropbox/Kasdin_Lab/MyPapers/EKF/rev2';
% cd(dirhome)

% cd ~/Dropbox/Kasdin_Lab/simulations

pitch = 300e-6; % meters
dx = pitch/13.02; %meters
xs = ((-N/2:N/2-1) + 1/2)*dx/pitch;


% Make a Gaussian
% xs1 = ( -floor(width1):1:floor(width1) );%*dx1;
% [XS1,YS1] = meshgrid(xs1);
% sigma1 = 1;%11.5;
% inf0 = exp(-4*log(2)*((XS1/sigma1).^2 + (YS1/sigma1).^2));



% dx1 = 13e-6;%cosd(mountAngDegx)*pitch/sampperact;
% dy1 = 13e-6;%cosd(mountAngDegy)*pitch/sampperact;
sigma = 1.28;
% infSampling = 5; % Minimum is ~5. This is the number of actuators across that the 2-D influence function will be.
% width1y = N;%2*round(infSampling*sampperact/2)+1; % pixels
% width1x = N;%2*round(infSampling*sampperact/2)+1; % pixels

% Make the influence function here.
xs1 = xs;%( -floor(width1x/2):1:floor(width1x/2) )*dx;
ys1 = xs;%( -floor(width1y/2):1:floor(width1y/2) )*dx;
[XS1,YS1] = meshgrid(ys1,xs1);
inf0 = exp(-4*log(2)*((XS1/(sigma*1)).^2 + (YS1/(sigma*1)).^2));



%%
% % xhat = lsqnonlin(@(xvar) IpC-makeGaussian(xvar,width1),xvar0) % Very finicky
options = optimset('Display','iter','TolX',1e-16);
% xhat = fminsearch(@(xvar) myFunc(xvar,width1,IpC),xvar0,options)
% c0 = 1e-5;
sigmaBest = fminbnd(@(sig) func_cost_for_BMC_inf_fit(sig,infMeas),1,2,options);
fprintf('Best fit FWHM is %.3f\n',sigmaBest)
% figure(9); imagesc(((cHat*ImeasCrop-IpC))); axis xy equal tight; ch = colorbar; colormap parula;
%%
infBestFit = funcGaussianForBMC(sigmaBest);
figure; imagesc(xs,xs,infMeas);axis xy equal tight; colorbar;
figure; imagesc(xs,xs,infBestFit); axis xy equal tight; colorbar;
figure; imagesc(xs,xs,infMeas-infBestFit); axis xy equal tight; colorbar;
% %%
% xvar0 = [1e-6, sigma1];
% out = makeGaussian(xvar0,width1); whos out
% 
% ImeasCrop = Imeas(101-width1:101+width1,100-width1:100+width1);
% % figure; imagesc(log10(ImeasCrop));
% 
% % % xhat = lsqnonlin(@(xvar) IpC-makeGaussian(xvar,width1),xvar0) % Very finicky
% options = optimset('Display','iter','TolX',1e-16);
% % xhat = fminsearch(@(xvar) myFunc(xvar,width1,IpC),xvar0,options)
% c0 = 1e-5;
% cHat = fminbnd(@(c) myFunc2(c,ImeasCrop,IpC),1e-8,1e-5,options);
% fprintf('Best fit contrast value is %.2e\n',cHat)
% figure(9); imagesc(((cHat*ImeasCrop-IpC))); axis xy equal tight; ch = colorbar; colormap parula;
% 
% x = 1:2*width1+1;
% figure(7); plot(x,IpC(:,width1+1),x,IpC(width1+1,:),x,cHat*ImeasCrop(width1+1,:),x,cHat*ImeasCrop(:,width1+1));
% figure(8); imagesc(log10(ImeasCrop)); axis xy equal tight; ch = colorbar; colormap parula;
% 
% % fprintf('Gaussian best fit contrast value is %.2e\n',xhat(1))
% 
% % Gbest = makeGaussian(xhat,width1); 
% 
% % f = fit([xs1,xs1],IpC,'gauss1')
% 
% figure(6); imagesc(log10(abs(IpC))); axis xy equal tight; ch = colorbar; colormap parula;
% % figure(8); imagesc(log10(Gbest)); axis xy equal tight; ch = colorbar; colormap parula;
% % figure(9); imagesc(((Gbest-IpC))); axis xy equal tight; ch = colorbar; colormap parula;
% 
% % figure(7); plot(x,IpC(:,width1+1),x,IpC(width1+1,:),x,Gbest(width1+1,:));

