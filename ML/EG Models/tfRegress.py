'''
In a regression problem, the goal is to predict the output of a continous value, such as a price
or a probability. This session uses an Auto MPG dataset to build a model to predict fuel-efficiency
of late-1970s and early 1980s automobile.
The model is provided with a description of many automobiles from that time period. This
description includes features like: cylinders, displacement, horsepower and weight.
'''

from __future__ import absolute_import, division, print_function

import pathlib
import pandas as pd
import seaborn as sb
import matplotlib.pyplot as plt

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

dataset_path = keras.utils.get_file("auto-mpg.data", "https://archive.ics.uci.edu/ml/machine-learning-databases/auto-mpg/auto-mpg.data")

# Import the data using pandas
column_names = ['MPG', 'Cylinders', 'Displacement', 'Horsepower', 'Weight',
                'Acceleration', 'Model Year', 'Origin']
raw_dataset = pd.read_csv(dataset_path, names=column_names,
                          na_values="?", comment='\t',
                          sep=" ", skipinitialspace=True)

dataset = raw_dataset.copy()
# print(dataset.tail())
# Prints the last 4 rows of dataset

dataset.isna().sum()
# I assume the above command shows the row containing unknown (na) values
# and then the command below removes the row from the dataset
dataset = dataset.dropna()

# remove/'pop out' the origin column
origin = dataset.pop('Origin')

# Create a new column using the true/false of if origin = the respective country at
# that row. For example, origin == 1 is a boolean test that gives 0 at all rows
# where origin != 0 and 1 otherwise.
dataset['USA'] = (origin == 1)*1.0
dataset['Europe'] = (origin == 2)*1.0
dataset['Japan'] = (origin == 3)*1.0
# print(dataset.tail())
# You should see 3 new columns to replace Origin: USA, Europe and Japan.

# Split the dataset into a training set (80%) and a test set (remainder)
train_dataset = dataset.sample(frac=0.8, random_state=0)
# print(train_dataset.index)

# Drop the indices from the dataset that are being used in the training set
test_dataset = dataset.drop(train_dataset.index)
# print(test_dataset.index)

# Shows joint distributions of columns in the trainig set. You can see some
# interesting correclations begin to arise
sb.pairplot(train_dataset[["MPG", "Cylinders", "Displacement", "Weight"]], diag_kind="kde")
# plt.show()

# Shows overall statistics
train_stats = train_dataset.describe()
train_stats.pop("MPG")
# The above line is to separate the value that the model is seeking to predict.
# You should not really worry about the stats for that I guess?
train_stats = train_stats.transpose()
print(train_stats)

train_labels = train_dataset.pop('MPG')
test_labels = test_dataset.pop('MPG')

# At this point we might need to normalise data. The stats from the training set
# show that the continuous values operate on different size scales (decimals to
# thousands). The model MIGHT converge without normalisation, however, training
# would become more difficult

def norm(x):
    return (x - train_stats['mean']) / train_stats['std']
    # Looks like a simple Gaussian normalisation

normed_train_data = norm(train_dataset)
normed_test_data = norm(test_dataset)

def build_model():
    model = keras.Sequential([
        layers.Dense(64, activation=tf.nn.relu, input_shape=[len(train_dataset.keys())]),
        layers.Dense(64, activation=tf.nn.relu),
        layers.Dense(1)
    ])

    optimiser = tf.keras.optimizers.RMSprop(0.001)

    # mae -> mean absolute error
    model.compile(loss='mse',
                  optimizer=optimiser,
                  metrics=['mae', 'mse'])
    return model

# We are creating two models so we make them in functions
model = build_model()
model.summary()

# Take a batch of 10 examples from the training data and test
example_batch = normed_train_data[:10]
example_result = model.predict(example_batch)
print(example_result)
# Gives you ten output predictions for MPG

class PrintDot(keras.callbacks.Callback):
    # Callback function called at the end of every epoch
    def on_epoch_end(self, epoch, logs):
        if epoch % 100 == 0:
            print('')
        print('.', end='')

EPOCHS = 1000

history = model.fit(
    normed_train_data, train_labels,
    epochs=EPOCHS, validation_split=0.2,
    verbose=0, callbacks=[PrintDot()]
)

def plot_history(history):
    # Visualise the model's progress using the stats stored in the history object
    hist = pd.DataFrame(history.history)
    hist['epoch'] = history.epoch
    print(hist.tail())

    plt.figure()
    plt.xlabel('Epoch')
    plt.ylabel('Mean Abs. Error (MPG)')
    plt.plot(hist['epoch'], hist['mean_absolute_error'],
             label='Train Error')
    plt.plot(hist['epoch'], hist['val_mean_absolute_error'],
             label='Val Error')
    plt.legend()
    plt.ylim([0, 5])

    plt.figure()
    plt.xlabel('Epoch')
    plt.ylabel('Mean Square Error [$MPG^2$]')
    plt.plot(hist['epoch'], hist['mean_squared_error'],
             label='Train Error')
    plt.plot(hist['epoch'], hist['val_mean_squared_error'],
             label='Val Error')
    plt.legend()
    plt.ylim([0, 20])

    plt.show()

plot_history(history)

'''
The graphs show little improvement, and sometimes, degradation in the validation error after
~100 epochs. We will use one of keras callbacks to test whether improvement has happened and
if not, automatically stop the training.

EarlyStopping takes the arguments:
    - monitor: quantity to be monitored
    - min_delta: minimum change in the monitored quantity to qualify as an improvement (ie an
      absolute change less than this will count as no improvement).
    - patience: number of epochs with no improvement after which training will be stopped
    - verbose: verbosity mode
    - mode: one of auto, min or max. In min mode, training stops when monitor quantity stops
      decreasing. The same happens in max mode when the qty stops increasing. In auto, the mode
      if inferred.
    - baseline: baseline value for the monitored qty. Training will cease if model does not show
      improvement over the baseline
'''

model = build_model()
early_stop = keras.callbacks.EarlyStopping(monitor='val_loss', patience=10)
history = model.fit(
    normed_train_data, train_labels,
    epochs=EPOCHS, validation_split=0.2,
    verbose=0, callbacks=[early_stop, PrintDot()]
)
# On the Validation set, average error good to +/- 2MPG

plot_history(history)

loss, mae, mse = model.evaluate(normed_test_data, test_labels, verbose=0)
print("Testing set Mean Abs Error: {:5.2f} MPG".format(mae))

# test_predictions needs to be 1-Dimensional to plot bar chart -> flatten()
test_predictions = model.predict(normed_test_data).flatten()

plt.scatter(test_labels, test_predictions)
plt.xlabel('True Values [MPG]')
plt.ylabel('Predicted Values [MPG]')
plt.axis('equal')
plt.axis('square')
plt.xlim([0, plt.xlim()[1]])
plt.ylim([0, plt.ylim()[1]])
_ = plt.plot([-100, 100], [-100, 100])
plt.show()

error = test_predictions - test_labels
plt.hist(error, bins=25)
plt.xlabel("Prediction Error [MPG]")
plt.ylabel("Count")
plt.show()

'''
Some notes:
- MSE is a common loss fn used for regression problems
- MAE is a common evaluation metric for regression problems
- When numeric input features have values with different ranges, each feature
  should be scaled independently to the same range (eg to a normal dist.)
- If there is not much training data, one technique is to prefer a small network
  with few hidden layers to avoid overfitting
- Early stopping is a useful technique to prevent overfitting.
'''
