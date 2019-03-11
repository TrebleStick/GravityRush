import numpy as np
import keras

from keras.layers import Dropout, BatchNormalization, Flatten, Dense, Input, Conv1D
from keras.layers.convolutional import Conv1D, MaxPooling1D
from keras.models import Model
from keras.callbacks import EarlyStopping

from sklearn.model_selection import train_test_split


class G8Classifier:
    def __init__(self, input_data=None, input_labels=None, input_dims=None, epochs=300, batch_size=16):
        # Data
        self.input_data = input_data
        self.train_data = None
        self.test_data = None

        self.input_labels = input_labels
        self.f_labels = np.zeros(shape=(len(self.input_data), 1, 17))
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

        x = Conv1D(filters=1024, kernel_size=7,
                   padding='same', activation='relu')(inputs)
        x = BatchNormalization(x)
        x = MaxPooling1D((2, 2))(x)
        x = Dropout(0.5)(x)
        x = Conv1D(filters=512, kernel_size=7,
                   padding='same', activation='relu')(x)
        x = BatchNormalization(x)
        x = MaxPooling1D((2, 2))(x)
        x = Dropout(0.6)(x)
        x = Flatten(input_shape=self.input_dims)(x)
        x = Dense(128, activation='relu', activity_regularizer=keras.regularizers.l1(0.00005))(x)
        x = Dropout(0.4)(x)
        x = Dense(17, activation='softmax')(x)

        model = Model(inputs, x)
        print(model.summary())
        self.model = model
        return model

    def fit(self):
        print("Fitting...")
        self.train_data, self.test_data, self.train_labels, self.test_labels = train_test_split(self.input_data,
                                                                                                self.input_labels,
                                                                                                test_size=0.2,
                                                                                                shuffle=True)
        # self.train_data = np.reshape(self.train_data, (len(self.train_data), 1, 4, 540))
        print("Train Data Shape:", self.train_data.shape)
        # self.train_labels = np.reshape(self.train_labels, (len(self.train_labels), 1, 17))
        print("Train Labels Shape:", self.train_labels.shape)

        # self.test_data = np.reshape(self.test_data, (len(self.test_data), 1, 4, 540))
        # self.test_labels = np.reshape(self.test_labels, (len(self.test_labels), 1, 17))

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
        np.savetxt("Error.txt", (predictions-self.test_labels))


### GET INPUT DATA AND LABELS ###

# Create G8Classifier object

# build model

# fit model






