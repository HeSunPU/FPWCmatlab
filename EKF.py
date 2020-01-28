import scipy.io
import pylab
import numpy
from math import *
import scipy.misc
import scipy.sparse
import sys
import time
import matplotlib.ticker
#import HCIL_model # make sure this model accounts for alignments etc
from matplotlib import pyplot as plt
import socket
import time
from InitializationPY import *
from EKF_MAT2PY import mat2py, recvall


# G = numpy.concatenate([numpy.concatenate([numpy.real(G1),
# 	numpy.imag(G1)], axis=1), numpy.concatenate([numpy.real(G2), numpy.imag(G2)],
# 	axis=1)], axis=0).T # real matrix [Re(G1),Im(G1); Re(G2), Im(G2)], for dark hole area

# Only use G2 for control right now
G = numpy.concatenate([numpy.concatenate([numpy.real(G2), numpy.imag(G2)],
	axis=1)], axis=0).T
# G = numpy.concatenate([numpy.real(G2), numpy.imag(G2)],
# 	axis=1).T
print(G.shape)

## DM setting after creating the dark hole

u0 = command #numpy.concatenate([DM1command, DM2command])

## Correction on top of u0
u = u0*0.0

## Drift covariances (might be a bit off, but the filter is not very sensitive to this)
Qlarge = G.dot(numpy.eye(len(u))).dot(G.T)* drift_std**2 # this needs to be number of actuators available, not of full control

## The Extended Kalman Filter class which stores its state
class Filter:
    def __init__(self, index, x0 = numpy.zeros(2)):
        self.index = index #pixel index
        self.x_hat = numpy.array(x0) #electric field (state) estimate (real 2x1 vector with real and imag parts)

        # self.Q = drift_stats["covs"][self.index] #process noise covariance
        self.Q = Qlarge[2*index:2*index+2, 2*index:2*index+2]
        self.P = 16*self.Q #initial state covariance

    def advance(self, Gu, I):
        ## "predic" step
        E_hat = complex(self.x_hat[0] + Gu[self.index], self.x_hat[1] + Gu[self.index+G.shape[0]//2]) #closed loop electric field estimate (open loop field + Jacobian * controls)
        z_hat = abs(E_hat)**2 + i_inco_avg#*scale # intensity estimate
        z = I[self.index] # intensity measurement

        H = [2*Gu[self.index] + 2*self.x_hat[0], 2*Gu[self.index+G.shape[0]//2] + 2*self.x_hat[1]] #observation equation linearization

        ## Kalman gain:
        S = self.P.dot(H).dot(H) + (dark_curr)**2
        K = self.P.dot(H)/S

        ## "update" step
        self.x_hat = self.x_hat + K.dot(z - z_hat) #closed loop electric field estimate
        self.P = self.P - numpy.outer(K,H).dot(self.P) #electric field covariance

        ## "advance" step
        self.P = self.P + self.Q #covariance increases due to druft
        self.x_hat = self.x_hat*1.0 #mean remains the same (the drift is zero mean)

        return E_hat


## Computing EFC gain
M = G.T.dot(G)
EFC_gain = numpy.linalg.pinv(M + alpha*numpy.eye(len(M))).dot(G.T) #EFC function in runEFC


## Sample random wavefront drift and dithering
numpy.random.seed(0)
u_drift = numpy.zeros(n_act_drift)
u_drifts = []
dus = []

for j in range(T):
    ## Random walk drift
    u_drifts.append(u_drift) #change drift
    u_drift = u_drift + drift_std*(numpy.random.random(u_drift.shape)-0.5) # in nm?

    ## Zero mean dithering
    dus.append(numpy.random.normal(0, sigma_u, u.shape))


## Initialize filters
#I0, E0_init = mat2py(DM1command, u0 ,len(DM1command),G1.shape[1])
fs = [Filter(index,[numpy.real(E0_init[index]), numpy.imag(E0_init[index])]) for index in range(G.shape[0]//2)]



## Run closed loop
for j in range(T):

	cont_power = j>4
	drift_power = j>4

	I_OL, E_OL = mat2py(DM1command + drift_power*u_drifts[j], u0 ,len(DM1command),G1.shape[1])
	I, E = mat2py(DM1command + drift_power*u_drifts[j], u0 + u,len(DM1command),G1.shape[1])

	E_OL_split  = numpy.zeros(2*len(E_OL))
	E_OL_split[0:n_px] = numpy.real(E_OL)
	E_OL_split[n_px:2*n_px] = numpy.imag(E_OL)

	E_split  = numpy.zeros(2*len(E))
	E_split[0:n_px]  = numpy.real(E)
	E_split[n_px:2*n_px]= numpy.imag(E)
	# plt.plot(u0+u, label="DM2 EFC+Dither+DH")
	# plt.plot(u0, label="DM2 DH command")
	# plt.plot(u, label="DM2 EFC+Dither")
	# plt.plot(DM1command, label="DM1 DH command")
	# plt.plot(u_drifts[j], label="DM1 Drift")
	# plt.legend()
	# plt.show()
	# print("Iteration %d: avg closed loop intensity %.01f, avg open loop intensity %.01f"%(j, numpy.linalg.norm(E)**2/scale, numpy.linalg.norm(E0)**2/scale))

	# print("Iteration %d: avg closed loop intensity %.01f"%(j, numpy.linalg.norm(I)/scale))

	# I = measurement(numpy.abs(E)**2) # get from getImage in lab, dark hole area only
	I2D_temp = numpy.zeros(99*83)
	# print('red',len(I2D_temp))
	I2D_temp[pixel_indices] = I
	I2D = I2D_temp.reshape(99,83)
	# print(I2D.shape)



	Gu = G.dot(u) #precompute the difference between closed and open loop fields
	Iperf = numpy.abs(E)**2
	E_hat_hat = []
	for f in fs:
			f.advance(Gu,I) #advance the filters
			# E_hat_hat.append(f.advance(Gu,Iperf+dark_curr))

    # Get field estimate
	x_hats = numpy.array([f.x_hat for f in fs])
	E_hat = numpy.concatenate([x_hats[:,0], x_hats[:,1]])


	# x_hat_hat = numpy.array(E_hat_hat)
	# print(len(E_hat_hat))
	# x_hathat = numpy.concatenate([x_hat_hat, x_hat_hat[:,1]])

	#print("mean estimated intensity")
	# E_hat2 = numpy.concatenate([numpy.real(E0), numpy.imag(E0)]) #for debugging only

	# print(numpy.linalg.norm(E_hat-E_hat2)/numpy.linalg.norm(E_hat2))

	## Record iteration data
	I_arr.append(I)
	I_OL_arr.append(I_OL)
	u_arr.append(u)
	E_hat_arr.append(E_hat)
	E_arr.append(E)
	E_OL_arr.append(E_OL)

	print("Iteration: ",j, "\navg closed loop intensity: ", numpy.mean(I),
		"\navg open loop intensity: ", numpy.mean(I_OL), "\nmean estimated OL intensity: ",
		numpy.mean(E_hat[0:n_px]**2+ E_hat[n_px:2*n_px]**2))

	# fig = plt.figure()
    # self.ax1 = fig.add_subplot(1, 2, 1)
	deltaE = G2.T.dot(u)
	deltaE_split = numpy.zeros(2*len(deltaE))
	deltaE_split[::2] = numpy.real(deltaE)
	deltaE_split[1::2] = numpy.imag(deltaE)

	Gu_comb = Gu[0:n_px] + Gu[n_px:2*n_px]*complex(0,1)

	# plt.plot(-Gu,label='Split -Gu')
	# # plt.plot(-Gu_comb,label='-Gu combined')
	# plt.plot(E_OL_split-E_split,lw=3,alpha = 0.5,label='Split OL-CL')
	# plt.plot(-G2.T.dot(u),label='-G2u')
	# plt.plot(E_OL-E,lw=2,alpha = 0.3,label='OL-CL')
	# plt.plot(E_hat,label = 'Ehat')
	# plt.plot(E_OL_split,lw=2,alpha = 0.5,label = "E_OL")

	# plt.plot(E,label = 'E')
	# plt.plot(E_hat_hat,lw=2,alpha = 0.5,label = "E_hat_hat")
	# plt.legend()
	# plt.title("Iteration %d"%(j))
	# plt.show()

    # Compute next command (EFC + dithering)

	u = dus[j] - cont_power*EFC_gain.dot(E_hat)
	# u = -EFC_gain.dot(E0) + dus[j]
	# ADD A SEND DM COMMAND

	#
	# print("Iteration %d: avg closed loop intensity %.01f, \
	# 	avg open loop intensity %.01f"%(j, numpy.mean(I), \
	# 	numpy.mean(I_OL)))


print("saving data")
to_save = ["I_arr","I_OL_arr","E_hat_arr","u_arr", "dus", "u_drifts","pixel_indices","i_inco_avg","dark_curr"]
scipy.io.savemat(fname, dict((s, numpy.array(eval(s))) for s in to_save))
print("data saved")
