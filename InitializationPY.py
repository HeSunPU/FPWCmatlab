import scipy.io
import pylab
import numpy
#import HCIL_model # make sure this model accounts for alignments etc

## File location and names for model and dark hole data
path = "C:/Lab/FPWCmatlab/dataLibrary/20191128/"
data_file = 'efcbatchSPC20191128Trial1'
model_file = 'model1'
runTrial  = '3'
## Import mat files with dark hole data
full_data = scipy.io.loadmat(path+data_file+'.mat')
full_model = scipy.io.loadmat(path+model_file+'.mat')


## Constants
# scale = 1.81e-8 #scaling factor for the electric field to photons (magic number, might need to calibrate?)
# scale = 1.81e-5 #scaling factor for the electric field to photons (magic number, might need to calibrate?)
targetFlux = full_data[data_file]['targetFlux'][0,0].reshape(-1)
camExpDig = full_data[data_file]['camExp'][0,0].reshape(-1)
camExpMaint = 0.2
scale0 = numpy.sqrt(1/((targetFlux)*camExpDig))
scale = numpy.sqrt(1/((targetFlux)*camExpMaint))
#scale = numpy.sqrt(1/((1.8780e+09)*0.5)) #ADJUST TO USE TARGET FLUX AND EXPOSURE TIME



alpha = 300000#30000 # controller alpha
sigma_u = 0.01#2e-4 #dither magnitude
dark_curr_cam = 12.0# CHANGE THIS TO READ FROM DATA 0.25 #dark current intensity in photons/pixel/frame (SUSAN LOOK INTO THIS)
i_inco_avg = 13.5# dark_curr_cam + 1.37 #10**-9 / scale**2 figure out why cant use 10**-9/scale**2
dark_curr = 500.0# CHANGE THIS TO READ FROM DATA 0.25 #dark current intensity in photons/pixel/frame (SUSAN LOOK INTO THIS)



T = 50#time steps


drift_std = 1e-2#1e-2#1e-4 standard deviation of drift DM command



## Building the Jacobian matrix:
G1_d = full_model['model']['G1'][0,0]
G2_d = full_model['model']['G2'][0,0]
G1 = G1_d.T/scale
G2 = G2_d.T/scale

## Dark Hole area

pixel_indices = full_data[data_file]['pixelIndex'][0,0].reshape(-1) - 1 #python is zero indexed
n_px = len(pixel_indices)
# pixel_indices = full_data[data_file]['pixelIndex'][0,0] - 1 #python is zero indexed

## DM setting after creating dark hole
# itr = full_data[data_file]['itr']
DMcommand_full = full_data[data_file]['DMcommand'][0,0]
n_act = int(DMcommand_full.shape[0]/2) # number of actuators per DM
DM1command = DMcommand_full[0:n_act,-1]
DM2command = DMcommand_full[n_act:2*n_act,-1]

command = DM2command # initial control command, set to be control DM command

n_act_cont = n_act
n_act_drift = n_act
## Final electric field estimate from dark hole, initial Efield estimate for maintenance
E_dh_full = full_data[data_file]['EfocalEst'][0,0]
# get correct initial electric field from initialization
#E0_init = E_dh_full[:,-1]/scale0
E0_init = E_dh_full[:,-1].reshape(-1)/scale0


## Initial Focal Plane Image
image_dh = full_data[data_file]['I'][0,0][:,:,-1]
# pylab.imshow(I_dh)
# pylab.show()


## Initialization of things to save
fname = data_file + "_drift" + runTrial
I_arr = []
I_OL_arr = []
u_arr = []
E_OL_arr = []
E_arr = []
E_hat_arr = []


##
#print(n_px*2)
# I2D_temp = numpy.zeros(83*99)
# print(command.shape)
#
#
# # to_save = ["E0_init","pixel_indices"]
# #
# # scipy.io.savemat(fname, dict((s, numpy.array(eval(s))) for s in to_save))
# #
# print(numpy.mean(G1))
# print(image_dh[pixel_indices.reshape(-1)].shape)
# print(E0_init.shape, len(pixel_indices), numpy.amax(pixel_indices),I2D_temp.shape)

# I2D_temp[pixel_indices.reshape(-1)] = E0_init
# I2D = I2D_temp.reshape(99,83)
# pylab.imshow(I2D.T)
# pylab.show()
