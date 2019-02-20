import scipy.io as sio
import numpy as np

from keras.models import Sequential
from keras.layers import Dense, Dropout


mat_contents = sio.loadmat('g8data2.mat')
mat_data = mat_contents['signalArr']
mat_labels = mat_contents['signalLabel']

print("No. of data: {}, No. of labels: {}".format(str(mat_data.shape), str(mat_labels.shape)))
train_data_num = int(0.85*len(mat_labels))
print("Train Data Number:", train_data_num)

train_data = mat_data[:train_data_num]
print("Train Data shape:", train_data.shape)

train_labels = mat_labels[:train_data_num]
print("Train Labels shape:", train_labels.shape)

test_data = mat_data[train_data_num:]
test_labels = mat_labels[train_data_num:]

train_data = train_data.reshape(178, 50)
test_data = test_data.reshape(test_data.shape[0], 50)

# Generate dummy data
#x_train = np.random.random((1000, 20))
#y_train = np.random.randint(2, size=(1000, 1))
#x_test = np.random.random((100, 20))
#y_test = np.random.randint(2, size=(100, 1))

model = Sequential()
model.add(Dense(64, input_dim=50, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(64, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(1, activation='sigmoid'))

model.compile(loss='binary_crossentropy',
              optimizer='rmsprop',
              metrics=['accuracy'])

model.fit(train_data, train_labels,
          nb_epoch=20,
          batch_size=128)

score = model.evaluate(test_data, test_labels, batch_size=128)


import coremltools
coreml_model = coremltools.converters.keras.convert(model)
coreml_model.save('zmodel.mlmodel')
