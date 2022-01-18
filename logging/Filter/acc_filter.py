'''
3D accelerometer filtering with median and low pass filter
@author: Kemeng Chen

Modified by Gizem.
'''
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from util import*


def test_data(file_name):
	cur_dir=os.getcwd()
	fs=512
	cutoff=10
	cutoff_fs = cutoff / fs
	print("cutoff_fs:", cutoff_fs)
	# file_path=os.path.join(cur_dir, '../data', file_name)
	file_path=os.path.join(cur_dir, file_name)
	# data=read_data(file_path, [1,2,3])
	# data=read_data_xlsx(file_path, [1,2,3])
	data=read_data_xlsx(file_path)
	plot_lines(data, fs, 'Raw data')
	# fft_plot(data, fs, 'Raw data')
	median_data=median_filter(data, 155)
	lpf_data=freq_filter(data, 155, 0.01953125)
	comb_data=freq_filter(median_data, 155, 0.01953125)
	# plot_lines(median_data, fs, 'median filter')
	# plot_lines(lpf_data, fs, 'low pass filter')
	# plot_lines(comb_data, fs, 'median+low pass filter')
	# fft_plot(lpf_data, fs, 'low pass filter')
	# fft_plot(median_data, fs, 'median filter')
	# fft_plot(comb_data, fs, 'median+low pass filter')
	# plot3D(data, 'raw data')
	# plot3D(median_data, 'median filter')
	# plot3D(lpf_data, 'low pass filter')
	# plot3D(comb_data, 'median+low pass filter')
	# plot_subplot(data, 'Raw data')
	# plot_subplot(median_data, 'Median filter')
	# plot_subplot(lpf_data, 'Low pass filter')
	plot_subplot(comb_data, 'Median + Lowpass Filter')
	plt.show()

if __name__ == '__main__':
	if len(sys.argv)<2:
		raise ValueError('No file name specified')
	test_data(sys.argv[1])