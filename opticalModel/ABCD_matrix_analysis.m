% Fiber mode diameter in mm.
f_md=0.004; % 4 microns
lam=0.000632; % Laser wavelength in mm.

w_beam=f_md/4; % w parameter is a radius with 1/e in amplitude. Fiedld mode diameter is 1/e^2.

q_beam=(-1.0i*lam/(pi*(w_beam^2)))^(-1);   % Gaussian beam parameter at the exit of the fiber


d_fiber_lens_1=9;   % Distance from the fiber to the first lens, 10 is nominal
f_lens_1=10;         % Focal distance of the lens 1
d_lens_1_to_lens_2=110; % Distance between lens 1 and lens 2
f_lens_2=100;       % Focal distance of the lens 2

d_lens_2_to_pinhole=100; % Distance from lens 2 to "pinhole" entrance to the telescope imaging system

d_pinhole_to_mirror=1600; % Distance from the pinhole to the spherical mirror

R_spherical_mirror=3200;  % Radius of curvature of the spherical mirror


M_1=[1 d_fiber_lens_1; 0 1];        %Propagation fiber to lens 1
M_2=[1 0; -1/f_lens_1 1];           %Refraction in the lens 2
M_3=[1 d_lens_1_to_lens_2; 0 1];    % Propagation lens 1 to lens 2
M_4=[1 0; -1/f_lens_2 1];           % Refraction in the lens 2
M_5=[1 d_lens_2_to_pinhole; 0 1];   % Propagaion to pinhole
M_6=[1 d_pinhole_to_mirror; 0 1]; % Propagation pinhole to mirror
M_7=[1 0; -2/R_spherical_mirror 1]; % Reflection in the spherical mirror
M_8=[1 d_pinhole_to_mirror; 0 1];   % Propagation spherical mirror to the DM1

M_before_tel=M_5*M_4*M_3*M_2*M_1;
q_telescope=(M_before_tel(1,1)*q_beam+M_before_tel(1,2))/(M_before_tel(2,1)*q_beam+M_before_tel(2,2));


M_total=M_8*M_7*M_6*M_5*M_4*M_3*M_2*M_1;
q_beam_after=(M_total(1,1)*q_beam+M_total(1,2))/(M_total(2,1)*q_beam+M_total(2,2));


b_waist=(-(lam/pi)*(imag(1/q_beam_after)^(-1)))^0.5;
b_curv=real(1/q_beam_after);

disp('Beam diameter at deformable mirror 1/e2 parameter in mm')
disp(4*b_waist)

disp('Beam curvature at deformable mirror')
disp(b_curv)
%%
close all
center = [5,5];
diameter = 10; % [mm]

rowV = [0:10/(DM.DMmesh(1)-1):diameter] - center(1); % 0 to diameter of DM
colV = [0:10/(DM.DMmesh(2)-1):diameter] - center(2); 

[rows,cols] = ndgrid(rowV,colV);
r = (rows.^2 + cols.^2).^0.5;
n = 1; %ind of refraction of air
E0 = 1;
wz = b_waist;
Rz = b_curv;
k = 2*pi*n/lam;
z = 0;


E_rz = E0/wz * exp(-r.^2 / wz.^2) .* exp(-1i*(b_curv*k*r.^2 / 2));
w0 = 1/max(max(E_rz));
E_rz = E_rz*w0;

% E_rz(~DM.DMstop) = 0;

x = [-diameter/2,diameter/2];
y=x;
% imagesc(x,y,E_rz)
% imagesc(x,y,real(E_rz))
imagesc(x,y,abs(E_rz))
colorbar
caxis([0.9,1])
hold on

th = 0:pi/50:2*pi;
xcirc = diameter/2 * cos(th) + 0;
ycirc = diameter/2 * sin(th) + 0;
plot(xcirc,ycirc,'m',0,0,'rp')
% plot(rows(round(r,2)==5.0),cols(round(r,2)==5.0),'m.')
hold off


% 