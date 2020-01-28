import numpy as np
import socket
import time

def recvall(soc,n):
    d=soc.recv(n)
    while len(d)<n:
        d+=soc.recv(n-len(d))
    return d

def mat2py(u1, u2, n_acts,n_pix):
    while True:
        try:
            soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            soc.connect(("127.0.0.1", 4651))

            # soc.send(u_tot[:n_acts].astype(dtype=">d").tobytes()) #send example
            # soc.send(u_tot[n_acts:].astype(dtype=">d").tobytes()) #send example
            soc.send(u1.astype(dtype=">d").tobytes()) #send example
            soc.send(u2.astype(dtype=">d").tobytes()) #send example

            # [len(DM1command):]
            #more code...

            # E_real = np.frombuffer(recvall(soc,n_pix*8), dtype=">d").reshape(n_pix) #receive example
            # E_imag = np.frombuffer(recvall(soc,n_pix*8), dtype=">d").reshape(n_pix) #receive example

            I = np.frombuffer(recvall(soc,n_pix*8), dtype=">d").reshape(n_pix) #receive example
            E_real = np.frombuffer(recvall(soc,n_pix*8), dtype=">d").reshape(n_pix)
            E_imag = np.frombuffer(recvall(soc,n_pix*8), dtype=">d").reshape(n_pix)
            soc.close()
            return I, E_real+E_imag*complex(0,1)
        except socket.error:
            print("Socket Error!")
            time.sleep(1)
            continue #retry if something went wrong



# class Filter:
#     def __init__(self, index, x0 = np.zeros(2)):
#         self.index = index #pixel index
#         self.x_hat = np.array(x0) #electrif field (state) estimate (real 2x1 vector with real and imag parts)
#
#         self.Q = drift_stats["covs"][self.index] #process noise covariance
#         self.P = 16*self.Q #initial state covariance
#
#     def advance(self, Gu, I):
#         ## "predic" step
#         E_hat = complex(self.x_hat[0] + Gu[self.index], self.x_hat[1] + Gu[self.index+G.shape[0]//2]) #closed loop electric field estimate (open loop field + Jacobian * controls)
#         z_hat = abs(E_hat)**2 + dark_curr*scale # intensity estimate
#         z = I[self.index] # intensity measurement
#
#         H = [2*Gu[self.index] + 2*self.x_hat[0], 2*Gu[self.index+G.shape[0]//2] + 2*self.x_hat[1]] #observation equation linearization
#
#         ## Kalman gain:
#         S = self.P.dot(H).dot(H) + (dark_curr*scale)**2
#         K = self.P.dot(H)/S
#
#         ## "update" step
#         self.x_hat = self.x_hat + K.dot(z - z_hat) #closed loop electric field estimate
#         self.P = self.P - np.outer(K,H).dot(self.P) #electric field covariance
#
#         ## "advance" step
#         self.P = self.P + self.Q #covariance increases due to druft
#         self.x_hat = self.x_hat*1.0 #mean remains the same (the drift is zero mean)
