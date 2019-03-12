#!/usr/bin/env python
# coding: utf-8

# In[94]:


# from keras.layers import Input, Conv2D, MaxPooling2D, UpSampling2D, BatchNormalization, Flatten
# from keras.models import Model
# from keras.callbacks import EarlyStopping

import scipy.io as sio
import numpy as np
import keras
from sklearn.model_selection import train_test_split
from keras.layers import Dropout, BatchNormalization, Flatten, Dense, Input, Conv1D
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.models import Model
from keras.callbacks import EarlyStopping
import matplotlib.pyplot as plt

import csv
split_step = 50
num_sensors = 2


# In[95]:


class G8Classifier:
    def __init__(self, input_data=None, input_labels=None, input_dims=None, epochs=300, batch_size=16):
        # Data
        self.input_data = input_data
        self.train_data = None
        self.test_data = None

        self.input_labels = input_labels
#         self.f_labels = np.zeros(shape=(len(self.input_data), 1, 17))
        self.train_labels = None
        self.test_labels = None

        # Dimensions
        self.input_dims = input_dims

        # Models
        self.decoder_model = None
        self.encoder_model = None
        self.net_model = None
        self.model = None

        self.epochs = epochs
        self.batch_size = batch_size

    def build_model(self):
        inputs = Input(shape=self.input_dims)

        x = Conv2D(filters=1024, kernel_size=3,
                   padding='same', activation='relu')(inputs)
        x = BatchNormalization(momentum=0.8)(x)
        x = MaxPooling2D((2,2), padding='same')(x)
        x = Dropout(0.5)(x)
        x = Conv2D(filters=512, kernel_size=3,
                   padding='same', activation='relu')(x)
        x = BatchNormalization(momentum=0.8)(x)
        x = MaxPooling2D((2,2),padding='same')(x)
        x = Dropout(0.6)(x)
        x = Flatten(input_shape=self.input_dims)(x)
        x = Dense(128, activation='relu', activity_regularizer=keras.regularizers.l1(0.00005))(x)
        x = Dropout(0.4)(x)
        x = Dense(4, activation='softmax')(x)

        model = Model(inputs, x)
        print(model.summary())
        self.model = model
        return model

    def fit(self):
        print("Fitting...")
        self.train_data, self.test_data, self.train_labels, self.test_labels = train_test_split(self.input_data/1023,
                                                                                                self.input_labels,
                                                                                                test_size=0.2,
                                                                                                shuffle=True)
        self.train_data = np.reshape(self.train_data, (len(self.train_data), 1, 2, int(1000/split_step)))
        print("Train Data Shape:", self.train_data.shape)
        self.train_labels = np.reshape(self.train_labels, (len(self.train_labels)))
        print("Train Labels Shape:", self.train_labels.shape)

        self.test_data = np.reshape(self.test_data, (len(self.test_data), 1, 2, int(1000/split_step)))
        self.test_labels = np.reshape(self.test_labels, (len(self.test_labels)))

        self.model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['acc'])
        e_stop = EarlyStopping(monitor='val_loss', patience=50)
        self.model.fit(self.train_data, self.train_labels,
                       epochs=self.epochs,
                       batch_size=self.batch_size,
                       validation_data=(self.test_data, self.test_labels),
                       callbacks=[e_stop])
        print("Model fitting complete")

        test_loss, test_acc = self.model.evaluate(self.test_data, self.test_labels)
        print('Test accuracy:', test_acc)

        predictions = self.model.predict(self.test_data)
#         np.savetxt("Error.txt", (predictions-self.test_labels))
#         print(predictions)


# In[96]:


# split_step = 2
# wrist_none_data_raw = np.concatenate((np.genfromtxt ('../EMG_data/ife_wrist_none.csv', delimiter=","), np.genfromtxt ('../EMG_data/ife_wrist_none_0.csv', delimiter=",")), axis=0 )

# wrist_none_data = np.zeros( (2, int(wrist_none_data_raw.shape[0]/2), int(1000/split_step)) )

# for i in range(0, wrist_none_data_raw.shape[0]):
#     wrist_none_data[i%2][int((i-i%2)/2)] = wrist_none_data_raw[i][0:1000:split_step]

# wrist_none_data = wrist_none_data.reshape(wrist_none_data.shape[1],wrist_none_data.shape[0], int(1000/split_step))
# print(wrist_none_data.shape)


# In[97]:



wrist_none_data_raw = np.concatenate((np.genfromtxt ('../EMG_data/ife_wrist_none.csv', delimiter=","), np.genfromtxt ('../EMG_data/ife_wrist_none_0.csv', delimiter=",")), axis=0 )

wrist_none_data = np.zeros( (2, int(wrist_none_data_raw.shape[0]/2), int(1000/split_step) ) )

for i in range(0, wrist_none_data_raw.shape[0]):
    wrist_none_data[i%2][int((i-i%2)/2)] = wrist_none_data_raw[i][0:1000:split_step]

wrist_none_data = wrist_none_data.reshape(wrist_none_data.shape[1],wrist_none_data.shape[0], int(1000/split_step))
# print(wrist_none_data.shape)
# ---------------------------------------------------------------------------------#
clench_data_raw = np.concatenate((np.genfromtxt ('../EMG_data/ife_clench.csv', delimiter=","),np.genfromtxt ('../EMG_data/ife_clench_0.csv', delimiter=",")),axis=0)
clench_data = np.zeros( (2, int(clench_data_raw.shape[0]/2), int(1000/split_step) ) )

for i in range(0, clench_data_raw.shape[0]):
    clench_data[i%2][int((i-i%2)/2)] = clench_data_raw[i][0:1000:split_step]

clench_data = clench_data.reshape(clench_data.shape[1],clench_data.shape[0], int(1000/split_step))
# print(clench_data.shape)
# ---------------------------------------------------------------------------------#
wrist_in_data_raw = np.concatenate((np.genfromtxt ('../EMG_data/ife_wrist_in.csv', delimiter=","),np.genfromtxt ('../EMG_data/ife_wrist_in_0.csv', delimiter=",")),axis=0)
wrist_in_data = np.zeros( (2, int(wrist_in_data_raw.shape[0]/2), int(1000/split_step) ) )

# print(wrist_in_data_raw.shape, wrist_in_data.shape)

for i in range(0, wrist_in_data_raw.shape[0]):
    wrist_in_data[i%2][int((i-i%2)/2)] = wrist_in_data_raw[i][0:1000:split_step]

wrist_in_data = wrist_in_data.reshape(wrist_in_data.shape[1],wrist_in_data.shape[0], int(1000/split_step))
# print(wrist_in_data.shape)

# ---------------------------------------------------------------------------------#
wrist_out_data_raw = np.concatenate((np.genfromtxt ('../EMG_data/ife_wrist_out.csv', delimiter=","),np.genfromtxt ('../EMG_data/ife_wrist_out_0.csv', delimiter=",")),axis=0)
wrist_out_data = np.zeros( (2, int(wrist_out_data_raw.shape[0]/2), int(1000/split_step) ) )

# print(wrist_out_data_raw.shape, wrist_out_data.shape)

for i in range(0, wrist_out_data_raw.shape[0]):
    wrist_out_data[i%2][int((i-i%2)/2)] = wrist_out_data_raw[i][0:1000:split_step]

wrist_out_data = wrist_out_data.reshape(wrist_out_data.shape[1],wrist_out_data.shape[0], int(1000/split_step))
# print(wrist_out_data.shape)

#-------------------------------FORMAT--DATA--FOR-TRAINING-------------------------------------#

X_data_raw = np.concatenate((wrist_none_data, wrist_in_data, wrist_out_data, clench_data), axis=0)
y_data_raw = np.concatenate( ( np.full((wrist_none_data.shape[0]), 0), np.full((wrist_in_data.shape[0]), 1), np.full((wrist_out_data.shape[0]), 2), np.full((clench_data.shape[0]), 3)), axis=0)

print('Training data shape:', X_data_raw.shape)

# X_train, X_test, y_train, y_test = train_test_split(X_data_raw, y_data_raw, test_size=0.20, random_state=42)


# In[98]:


g8_model = G8Classifier(X_data_raw, y_data_raw, (1,X_data_raw.shape[1], X_data_raw.shape[2]), epochs=30)
g8_model.build_model()
g8_model.fit()


# In[99]:


g8_model.model.save_weights('gooood_model_split_'+str(split_step)+'.h5') 


# In[ ]:




