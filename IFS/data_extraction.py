import scipy.io as sio
import scipy.signal as signal
import matplotlib.pyplot as plt
import numpy as np

defocus_images = sio.loadmat('data_defocus_monochromatic.mat')
stage_pos = defocus_images['data_defocus_monochromatic'].item()[0]
images = defocus_images['data_defocus_monochromatic'].item()[1]

dark = sio.loadmat('dark.mat')
dark = dark['dark']

dark = 1278.9

current_image = images[:, :, 11, 0]
current_image[current_image>1500] = dark

current_image_sub_dark = current_image - dark

kernel = np.array([[20, 50, 20], [50, 110, 50], [20, 50, 20]])
image_filtered = signal.convolve2d(current_image_sub_dark, kernel, mode='same')

#%%
image_defocus = sio.loadmat('img_defocus.mat')['img_defocus'][:, :, 1::, :]
stage_pos = np.arange(5.6, 7.1, 0.1)
