import scipy.io as sio
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout
#import keras
import tensorflow as tf

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

model = Sequential()
#[
# keras.layers.Dense(128, activation=tf.nn.relu),
# keras.layers.Dropout(0.25),
# keras.layers.Dense(64, activation=tf.nn.relu),
# keras.layers.Dense(2, activation=tf.nn.softmax),
# ]

model.add(Dense(128, activation='relu', ))
model.add(Dropout(0.25))
model.add(Dense(64, activation='relu'))
model.add(Dense(2, activation='softmax'))


model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

early_stop = keras.callbacks.EarlyStopping(monitor='val_loss', patience=10)
model.fit(train_data,
          train_labels,
          epochs=20,
          validation_split=0.3,
          callbacks=[early_stop])
print('Model fitting complete')

test_labels = (test_labels + 1) % 2
test_loss, test_acc = model.evaluate(test_data, test_labels)
print('Test accuracy:', test_acc)

# model.save("g8mlzulu.h5")

import coremltools
coreml_model = coremltools.converters.keras.convert(model)
coreml_model.save('model.mlmodel')
#model_json = model.to_json()
#with open("model.json", "w") as json_file:
#    json_file.write(model_json)
#model.save_weights("model.h5")
#print("Saved model to disk")
