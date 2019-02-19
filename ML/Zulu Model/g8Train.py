import scipy.io as sio
import scipy.signal as signal
from tensorflow import keras
import tensorflow as tf
import numpy as np

import matplotlib.pyplot as plt

# sEMG signals from this database sampled at 2kHz
def prog_bar(curr, total, full):
    frac = curr/total
    filled = round(frac*full)
    print('\r' + '[' + '='*filled + '.'*(full-filled) + ']', '({:>7.2%})'.format(frac), end='')

def norm_data(x, img_length):
    norm_x = np.zeros((2, img_length))
    norm_x[0] = (x[0] - np.mean(x[0]))/np.std(x[0])
    norm_x[1] = (x[1] - np.mean(x[1]))/np.std(x[1])
    return norm_x

def get_features(x, train_num):
    print("Getting features...")
    # Initialise matrix for holding all feature vector components
    f_mat = np.zeros((train_num, 10, 2))
    img_length = len(x[1, 1, :])

    for arr in range(train_num):
        # norm_arr = norm_data(x[arr, :, :], img_length)
        f_mat[arr, 0:2, :] = get_avgs(x[arr, :, :], img_length)      # Should return a 2x2 array with mav and rms for that image
        # f_mat[arr, 2, :] = get_zero_cross(x[arr, :, :], img_length)
        f_mat[arr, 2, :] = get_variance(x[arr, :, :])
        f_mat[arr, 3:5, :] = get_markers(x[arr, :, :])
        f_mat[arr, 5:7, :] = get_range(x[arr, :, :])
        f_mat[arr, 7:9, :] = get_slope_data(x[arr, :, :], img_length)
        f_mat[arr, 9, 1] = get_power(x[arr, 0, :])
        f_mat[arr, 9, 2] = get_power(x[arr, 1, :])

        prog_bar(arr, train_num, 25)

    print('\n')
    print(f_mat[1200, :, :])
    return f_mat

def get_avgs(x_in, img_length):
    #  mav
    x_mav = np.mean(np.absolute(x_in), axis=1)
    # print("MAV:", x_mav)

    # rms
    pow2x_in = np.power(x_in, 2)
    rms = np.sum(pow2x_in, axis=1)/img_length
    rms = np.power(rms, 0.5)
    # print("RMS:", rms)
    return x_mav, rms

# def get_zero_cross(x_in, img_length):
#     # print("Counting zero crossings")
#     zero_count = [1, 1]
#     for _ in range(img_length-1):
#         if x_in[0, _]*x_in[0, _+1] < 0:
#             zero_count[0] += 1
#         if x_in[1, _] * x_in[1, _ + 1] < 0:
#             zero_count[1] += 1
#     print("Zero count:", zero_count)
#     return zero_count

def get_variance(x_in):
    # print("Getting variance")
    var = np.var(x_in, axis=1)
    # print("Var:", var)
    return var

def get_markers(x_in):
    # print("Getting markers")
    abs_x = np.absolute(x_in)

    # return abs(max) and abs(min)
    abs_max = np.amax(abs_x, axis=1)
    # print("Abs max:", abs_max)
    abs_min = np.amin(abs_x, axis=1)
    # print("Abs min:", abs_min)

    # return median
    # med = np.median(x_in, axis=1)
    # print("Med:", med)
    return_arr = np.vstack((abs_max, abs_min))
    # return_arr = np.vstack((return_arr, med))
    # print(return_arr)
    return return_arr

def get_range(x_in):
    # print("Getting ranges")
    # Give arithmetic range and geometric range
    x_max = np.amax(x_in, axis=1)
    x_min = np.amin(x_in, axis=1)

    a_range = np.subtract(x_max, x_min)

    for _ in range(len(x_min)):
        if x_min[_] == 0:
            x_min[_] = 0.0024
    g_range = np.divide(x_max, x_min)

    # print("Ranges:", a_range, g_range)
    return a_range, g_range

def get_slope_data(x_in, img_length):
    # print("Getting slope data")
    slope_change = [1, 1]
    # slope_mag = np.zeros((2, len(x_in[1, :])))
    current_grad = [0, 0]
    max_slope_change = [0, 0]

    # Slope change count
    # Slope change magnitude
    for _ in range(img_length - 1):
        # if _ < len(x_in[1, :])- 2:
        #     slope_mag[0, _ + 1] = x_in[0, _ + 1] - x_in[0, _]
        #     slope_mag[1, _ + 1] = x_in[1, _ + 1] - x_in[1, _]

        if x_in[0, _+1]-x_in[0, _] != current_grad[0]:
            slope_change[0] += 1
            current_grad[0] = x_in[0, _+1]-x_in[0, _]
            if current_grad[0] > max_slope_change[0]:
                max_slope_change[0] = current_grad[0]
        if x_in[1, _+1]-x_in[1, _] != current_grad[1]:
            slope_change[1] += 1
            current_grad[1] = x_in[1, _+1]-x_in[1, _]
            if current_grad[1] > max_slope_change[1]:
                max_slope_change[1] = current_grad[1]
    # print("Slope Data:", slope_change, max_slope_change)
    return slope_change, max_slope_change

def get_power(x):
    fs = 2000
    freq, p_den = signal.periodogram(x, fs)
    freq = np.amax(freq)
    p_den = np.mean(p_den)
    return freq, p_den

def get_fft(x):
    return np.fft.fft(x)

# def get_dwt(x, img_length):



mat_contents = sio.loadmat('DB1_sigData.mat')
mat_data = mat_contents['signalArr']
mat_labels = mat_contents['signalLabel']
mat_labels -= 1     # label values need to start from 0

# f, p = get_power(mat_data[2447, 1, :])
# print(f)
# print(p)
# plt.plot(f, p)
# plt.show()

# out = get_fft(mat_data[2448, 1, :])
# print(out.shape)
# print(np.amax(out))
# plt.plot(out)
# plt.show()

'''
Feature matrix consists of features for each segment including:
 1) Mean Absolute Value
 2) Root Mean Square
 3) Zero Cross Count - looks useless
 4) Variance
 5) Absolute Maximum
 6) Absolute Minimum - seems to be zero so remove
 7) Absolute Range
 8) Geometric Range
 9) Median - Seems useless
 10) Slope Change Count - Try without
 11) Max. Slope Change - Try without
 -----------
 EXTRA SPICE:
 - Marginal discrete wavelet transform
 - FFT
 - Power Spectrum (dominant frequency and mean)
'''

print("No. of train data: {}, No. of train labels: {}".format(str(mat_data.shape), str(mat_labels.shape)))
train_data_num = int(0.95*len(mat_labels))
print("Train Data Number:", train_data_num)

# Data needs to be of format (index of 2D array, 2D array of data)
train_data = mat_data[:train_data_num]
print("Train Data shape:", train_data.shape)

# train_features = get_features(train_data, train_data_num)
# print(train_features.shape)
# print(get_avgs(train_data[31, :, :], train_data_num))

# print("Single image array:", train_data[1788, :, :].shape)
# print(train_data[31, :, :])   # Check that you're looking at the right data
train_labels = mat_labels[:train_data_num]
print("Train Labels shape:", train_labels.shape)
# print(train_labels[1:100])

test_data = mat_data[train_data_num:]
# test_features = get_features(test_data, len(test_data[:]))
test_labels = mat_labels[train_data_num:]

model = keras.Sequential([
    keras.layers.Flatten(input_shape=(2, 568)),
    # keras.layers.LSTM(units=100, return_sequences=True, input_shape=(1, 1136)),
    # keras.layers.Dropout(0.1),
    keras.layers.Dense(128, activation=tf.nn.relu),
    keras.layers.Dropout(0.3),
    # High l2 regularisation drops model accuracy
    # keras.layers.Dense(128, activation=tf.nn.relu),
    # keras.layers.Dense(32, activation=tf.nn.relu),
    # keras.layers.Dropout(0.1),
    keras.layers.Dense(64, activation=tf.nn.relu),
    keras.layers.Dense(12, activation=tf.nn.softmax)
])
# 3 hidden layers does not provide significant test accuracy over 2
# Layer 1 with 128 neurons and layer 2 with 64 neurons seems to work well

model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

early_stop = keras.callbacks.EarlyStopping(monitor='val_loss', patience=10)
model.fit(train_data, train_labels,
          epochs=100,
          batch_size=256,
          validation_split=0.1,
          callbacks=[early_stop])
# Epochs refer to the feedforward and backpropagation cycles
print('Model fitting complete')

test_loss, test_acc = model.evaluate(test_data, test_labels)
print('Test accuracy:', test_acc)

# predictions = model.predict(test_data)





