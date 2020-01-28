import scipy.io
import pylab
import numpy
from math import *
import scipy.misc
import scipy.sparse
import sys
import time
import matplotlib.ticker
import HCIL_model # make sure this model accounts for alignments etc
from matplotlib import pyplot as plt
import socket
import time
from InitializationPY import *
from EKF_MAT2PY import mat2py, recvall

G = numpy.concatenate([numpy.concatenate([numpy.real(G2), numpy.imag(G2)],
	axis=1)], axis=0).T
'''
RUN WITH ZERO NOISE TO CHECK SCALE ONLY
'''
numpy.random.seed(0)

u = DM2command

u_probe = numpy.random.normal(0, 0.9, u.shape)

I,E = mat2py(DM1command, u,len(DM1command),G1.shape[1])
Ipos,Epos = mat2py(DM1command, u + u_probe,len(DM1command),G1.shape[1])
Ineg,Eneg = mat2py(DM1command, u - u_probe,len(DM1command),G1.shape[1])

Idiff = Ipos + Ineg - 2* I

# amplitudeSquare = (Ipos + Ineg) /2 - I
# amplitudeSquare[amplitudeSquare<0] = 0
# amplitude = numpy.sqrt(amplitudeSquare)

# Idiff_jac = 2*(numpy.real(G2.T)@u_probe)**2 + 2*(numpy.imag(G2.T)@u_probe)**2
# Idiff_jac = 2*(numpy.abs(G2.T)@u_probe)*(numpy.abs(G2.T)@u_probe)
Idiff_jac = 2*numpy.abs(G2.T.dot(u_probe))**2
# plt.plot(Idiff)
# # plt.plot(Idiff_jac)
# plt.plot(Idiff_jac/scale**2)
# plt.plot(Idiff/Idiff_jac)
print(numpy.median(Idiff/Idiff_jac))
scale = numpy.sqrt(numpy.median(Idiff_jac/Idiff))
scale2 = numpy.sqrt(1/((1.8780e+09)*0.5))

print(scale, scale2)
plt.plot(Epos-E,label='diffE')
# plt.plot(Idiff_jac)
plt.plot(G2.T.dot(u_probe),label='Gu')
plt.legend()
plt.show()

# plt.plot(Idiff)
# # plt.plot(Idiff_jac)
# plt.plot(Idiff_jac/scale**2)
# plt.show()
