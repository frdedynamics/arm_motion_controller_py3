'''
utility function to read data, integration, and plot
@author: Kemeng Chen

Modified by Gizem.
'''

import os
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.fftpack import fft
from scipy import signal
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

def read_data(file_path, columns):
	'''
	read files according to file_path and columns
	args:
		file_path, columns (list)
	return:
		data (numpy array)
	'''
	if not os.path.isfile(file_path):
		raise AssertionError(file_path, 'not found')
	mode='r'
	with open (file_path, mode) as f:
		lines=f.readlines()
		num_rows=len(lines)
		print((str(num_rows), ' rows'))
		num_cols=len(columns)
		print((str(num_cols), ' cols'))
		print(columns)
		data=np.zeros([num_rows, num_cols])
		for indice, line in enumerate(lines[3:]):
			row=line.rstrip().split('\t')
			# print(row)
			for ii, i in enumerate(columns):
				data[indice,ii]=row[i]
		# data[:,2]-=10
		# for i in range(3):
		# 	data[:,i]=calibration(data[:,i])
		print("data", data[0:5, 0:3])
	f.close()
	return data

def read_data_xlsx(file_path):
	'''
	read files according to file_path and columns
	args:
		file_path, columns (list)
	return:
		data (numpy array)
	'''
	if not os.path.isfile(file_path):
		raise AssertionError(file_path, 'not found')
	mode='r'
	xl = pd.ExcelFile(file_path)
	num_of_sheet = len(xl.sheet_names)
	imu_df = [0] * num_of_sheet
	for i in range(num_of_sheet):
		sheet_name = 'IMU' + str(i)
		imu_df[i] = xl.parse(sheet_name)
	num_rows, num_cols=imu_df[0].shape
	print("row:", num_rows)
	print("cols:", num_cols)
	acc_data = np.array(num_of_sheet*[np.empty([num_rows, 3])])
	quat_data = [0] * num_of_sheet
	measurement = [0] * num_of_sheet
	for i in range(num_of_sheet):
		acc_data[i] = np.empty([num_rows, 3])
		# print "acc0", acc_data[i].shape
		quat_data[i] = np.empty([num_rows, 4])
		acc_data[i] = imu_df[i][['acc_x','acc_y','acc_z']].to_numpy(dtype='float')
		# acc_data[i] = acc_data[i]*100
		# quat_data[i] = imu_df[i][['quat_w', 'quat_x', 'quat_y', 'quat_z']].to_numpy(dtype='float')
		# measurement[i] = imu_df[i][['meas_x','meas_y','meas_z']].to_numpy(dtype='float')
	measurement = np.array(measurement)
	return acc_data[0]

def median_filter(data, f_size):
	lgth, num_signal=data.shape
	f_data=np.zeros([lgth, num_signal])
	for i in range(num_signal):
		f_data[:,i]=signal.medfilt(data[:,i], f_size)
	return f_data

def freq_filter(data, f_size, cutoff):
	lgth, num_signal=data.shape
	f_data=np.zeros([lgth, num_signal])
	lpf=signal.firwin(f_size, cutoff, window='hamming')
	for i in range(num_signal):
		f_data[:,i]=signal.convolve(data[:,i], lpf, mode='same')
	return f_data

def fft_plot(data, fs, title):
	lgth, num_signal=data.shape
	fqy=np.zeros([lgth,num_signal])
	fqy[:,0]=np.abs(fft(data[:,0]))
	fqy[:,1]=np.abs(fft(data[:,1]))
	fqy[:,2]=np.abs(fft(data[:,2]))
	index=np.arange(int(lgth/2))/(int(lgth/2)/(fs/2))
	fig, ax=plt.subplots()
	labels=['x','y','z']
	color_map=['r', 'g', 'b']
	for i in range(3):
		ax.plot(index, fqy[0:int(lgth/2),i], color_map[i], label=labels[i])
	ax.set_xlim([0, fs/2])
	ax.set_xlabel('Hz')
	ax.set_title('Frequency spectrum: '+title)
	ax.legend()

def plot_lines(data, fs, title):
    num_rows, num_cols=data.shape
    if num_cols!=3:
        raise ValueError('Not 3D data')
    fig, ax=plt.subplots()
    labels=['x','y','z']
    color_map=['r', 'g', 'b']
    index=np.arange(num_rows)/fs
    for i in range(num_cols):
        ax.plot(index, data[:,i], color_map[i], label=labels[i])
    ax.set_xlim([0,num_rows/fs])
    ax.set_xlabel('Time [sec]')
    # ax.set_title('Time domain: '+title)
    ax.legend()

def acc_integration(data):
	num_rows, num_cols=data.shape
	int_data=np.zeros(data.shape)
	for i in range(num_cols):
		int_data[:,i]=TZ_integration(data[:,i])
	return int_data

def plot3D(data, title):
	fig=plt.figure()
	ax=fig.add_subplot(111, projection='3d')
	ax.plot(xs=data[:,0], ys=data[:,1], zs=data[:,2], zdir='z')
	# ax.set_title(title)

def plot_subplot(data, title, hold=True, dt=0.01, ylim=None):
    num_rows, num_cols=data.shape
    index=np.arange(0,num_rows/(1/dt),dt)
    fig, ax = plt.subplots()
    # fig.suptitle(title)
    labels=['x','y','z','w']
    color_map=['r', 'g', 'b', 'y']
    # color_map=['r', 'w', 'w']
    for i in range(num_cols):
        ax.plot(index, data[:,i], color_map[i], label=labels[i])
    ax.set_xlim([0,num_rows/(1/dt)-dt])
    ax.set_ylim(ylim)
    ax.set_xlabel('Time [sec]')
    # ax.set_title('Time domain: '+title)
    ax.set_title(title)
    ax.legend()
    # plt.hold(hold)


def calibration(signal):
	inc_eng=np.sum(np.clip(signal, a_min=0, a_max=None))
	der_eng=-1*np.sum(np.clip(signal, a_max=0, a_min=None))
	if inc_eng==0 or der_eng==0:
		raise ValueError('Calibration rule does NOT hold')

	if inc_eng>der_eng:
		beta_p=1
		beta_n=inc_eng/der_eng
		c_signal=(beta_n-1)*np.clip(signal, a_max=0, a_min=None)+signal
	elif der_eng > inc_eng:
		beta_n = 1
		beta_p = der_eng/inc_eng
		c_signal = (beta_p-1)*np.clip(signal, a_min=0, a_max=None) + signal
	else:
		c_signal=signal
	return c_signal


def TZ_integration(in_signal):
	lgth = in_signal.shape
	integral=np.zeros(lgth)
	c=0
	for indice, s in enumerate(in_signal):
		if indice==0:
			integral[indice]=c+in_signal[indice]
		else:
			integral[indice]=integral[indice-1]+in_signal[indice]
	return integral