import tensorflow as tf
from tensorflow import keras

import numpy as np
import matplotlib.pyplot as plt

'''
The IMDB dataset comes packaged with TensorFlow. It has already been
preprocessed such that the reviews (sequences of words) have been
converted to sequences of integers, where each integer represents a
specific word in a dictionary.
'''

imdb = keras.datasets.imdb
(train_data, train_labels), (test_data, test_labels) = imdb.load_data(num_words=10000)
# The argument num_words=10000 keeps the top 10,000 most frequently
# occurring words in the training data. Rare words are discarded in
# order to minimise the size of data to be processed.

print("Training entries: {}, labels: {}".format(len(train_data), len(train_labels)))
# print(train_data[0])

# Be aware of the fact that movie reviews may be different lengths
print("Length of review 1: {}, Length of review 2: {}".format(len(train_data[0]), len(train_data[1])))

word_index = imdb.get_word_index()
# word_index is now a dict object (key-value pair) contains a mapping for integers to specific words

# The first indices are reserved for special words PAD, START, etc. as below
word_index = {k: (v+3) for k, v in word_index.items()}
word_index["<PAD>"] = 0
word_index["<START>"] = 1
word_index["<UNK>"] = 2 # Unknown word
word_index["<UNUSED>"] = 3

# print(word_index.keys()) -> Prints out all the keys in the dictionary

# Reverses key-value pairs
reverse_word_index = dict([(value, key) for (key, value) in word_index.items()])
# print(reverse_word_index)

def decode_review(text):
    return ' '.join([reverse_word_index.get(i, '?') for i in text])

# print(decode_review(train_data[0]))

train_data = keras.preprocessing.sequence.pad_sequences(train_data,
                                                        value=word_index["<PAD>"],
                                                        padding='post',
                                                        maxlen=256)

test_data = keras.preprocessing.sequence.pad_sequences(test_data,
                                                       value=word_index["<PAD>"],
                                                       padding='post',
                                                       maxlen=256)

# The reviews have been padded to be the same length = 256 (assumption is that no review
# is longer than 256 in length
print("Length of review 1: {}, Length of review 2: {}".format(len(train_data[0]), len(train_data[1])))

dict_size = 10000

model = keras.Sequential()
model.add(keras.layers.Embedding(dict_size, 16))
model.add(keras.layers.GlobalAveragePooling1D())
model.add(keras.layers.Dense(16, activation=tf.nn.relu))
model.add(keras.layers.Dense(1, activation=tf.nn.sigmoid))

# Summarises the layers of the model
model.summary()

'''
- The first layer is an Embedding layer. It takes the integer-encoded dictionary and looks up the embedding
  vector. The vectors are learned as the model trains. The vectors add a dimension to the output array:
  (batch, sequence, embedding). Embedding vectors are essentially a vector-based representation of words that
  can be used compare and distinguish between features of other vectors
- The second layer returns a fixed length output for each vector by averaging over the sequence dimension. This
  allows the model to handle inputs of variable length in a simple manner - but not necessarily the most reliable.
- The output from the pooling layer is sent through a dense layer of 16 neurons
- The last layer is densely connected with a single output neuron. Using the sigmoid activation function, its
  output is a float (probability) between 0 and 1.

NB - If a model had more hidden units and hidden layers, it is susceptible to learning unwanted patterns - ones that
     improve performance on training data but not test data ie overfitting.
'''

model.compile(optimizer='adam',
              loss='binary_crossentropy',
              metrics=['accuracy'])

# Create a validation set -> check the accuracy of the model on data it has not seen before
# Set apart 10,000 examples from the training data, which allows us to tune the model while training

x_val = train_data[:10000]
# x_val contains the first 10000 examples from train_data
partial_x_train = train_data[10000:]
# partial_x_train contains the rest of the training data

y_val = train_labels[:10000]
# x_val contains the first 10000 examples from train_data
partial_y_train = train_labels[10000:]
# partial_x_train contains the rest of the training data

# Trains the model for 40 epochs in batches of 512 samples
# Simultaneously, you monitor the model's loss and accuracy on the validation set
history = model.fit(partial_x_train,
                    partial_y_train,
                    epochs=40,
                    batch_size=512,
                    validation_data=(x_val, y_val),
                    verbose=1)

results = model.evaluate(test_data, test_labels)
print(results)

history_dict = history.history
# Gets the history object from the model.fit method. This object contains a history
# of training accuracy and loss and the validation accuracy and loss.
print("Dict. keys: ", history_dict.keys())

acc = history_dict['acc']
val_acc = history_dict['val_acc']
loss = history_dict['loss']
val_loss = history_dict['val_loss']

epochs = range(1, len(acc) + 1)

# 'bo' is for blue dots
plt.plot(epochs, loss, 'bo', label='Training loss')
# 'b' is for a solid blue line
plt.plot(epochs, val_loss, 'b', label='Validation loss')
plt.title('Training and Validation loss')
plt.xlabel('Epochs')
plt.ylabel('Loss')
plt.legend()

plt.show()
# We observe that validation loss is at a minimum after about 20 epochs. Further training -> overfitting

plt.clf()   # clear figure

plt.plot(epochs, acc, 'bo', label='Training acc')
plt.plot(epochs, val_acc, 'b', label='Validation acc')
plt.title('Training and validation accuracy')
plt.xlabel('Epochs')
plt.ylabel('Accuracy')
plt.legend()

plt.show()







