import tensorflow as tf
from tensorflow import keras

import numpy as np
import matplotlib.pyplot as plt

# Use 60,000 images to train the network and 10,000 to evaluate the accuracy

fashion_mnist = keras.datasets.fashion_mnist
(train_images, train_labels), (test_images, test_labels) = fashion_mnist.load_data()

'''
The above line loads four NumPy arrays.
    - train_images and train_labels: Training sets that the model uses to learn
    - test_images and tes_labels: Test sets that the model is tested against

Images are 28x28 NumPy arrays with pixel values ranging from 0 to 255.

Labels are an array of integers from 0 to 9. They correspond to the class(ification) that
the image represents ie which item of clothing:
    0 = T-shirt/top
    1 = Trouser
    2 = Pullover
    3 = Dress
    4 = Coat
    5 = Sandal
    6 = Shirt
    7 = Sneaker
    8 = Bag
    9 = Ankle Boot
Each image is mapped to a single label
'''

def plot_image(i, predictions_array, true_label, img):
    predictions_array, true_label, img = predictions_array[i], true_label[i], img[i]
    plt.grid(False)
    plt.xticks([])
    plt.yticks([])

    plt.imshow(img, cmap=plt.get_cmap('binary'))

    predicted_label = np.argmax(predictions_array)
    if predicted_label == true_label:
        color = 'blue'
    else:
        color = 'red'

    plt.xlabel("{} {:2.0f}% ({})".format(class_names[predicted_label],
                                         100 * np.max(predictions_array),
                                         class_names[true_label]),
               color=color)

def plot_value_array(i, predictions_array, true_label):
    predictions_array, true_label = predictions_array[i], true_label[i]
    plt.grid(False)
    plt.xticks([])
    plt.yticks([])

    thisplot = plt.bar(range(10), predictions_array, color='#777777')
    plt.ylim([0, 1])
    predicted_label = np.argmax(predictions_array)

    thisplot[predicted_label].set_color('red')
    thisplot[true_label].set_color('blue')

class_names = ['Top', 'Trouser', 'Pullover', 'Dress', 'Coat',
               'Sandal', 'Shirt', 'Sneaker', 'Bag', 'Boot']

print('Dimensions of training set: ', train_images.shape)
print('Number of labels in training set: ', len(train_labels))

'''
plt.figure()
plt.imshow(train_images[0])
plt.colorbar()
plt.grid(False)
plt.show()
# Plots a pixel image of the first training image (a show). Shows
# that pixels have values on a scale of 0 to 255 and need normalising.
'''

train_images = train_images / 255.0
test_images = test_images / 255.0

'''
plt.figure(figsize=(10, 10))
for i in range(30):
    plt.subplot(6, 5, i+1)
    # First two arguments relate to the number of images in each row and
    # column. In this image, there will be 6 rows and 5 columns
    plt.xticks([])
    plt.yticks([])
    plt.grid(False)
    plt.imshow(train_images[i], cmap=plt.cm.binary)
    plt.xlabel(class_names[train_labels[i]])
plt.show()
'''

model = keras.Sequential([
    keras.layers.Flatten(input_shape=(28, 28)),
    keras.layers.Dense(128, activation=tf.nn.relu),
    keras.layers.Dense(10, activation=tf.nn.softmax)
])

'''
- The first layer in this network transfroms the 28x28 array into a 784x1 array
- This layer is followed by two dense ones. These are densely-connected, but not
  necessarily fully-connected layers.
  -> The first dense layer has 128 neurons
  -> The second dense layer has 10 neurons. It is a softmax layer. It returns an
     array of 10 probability scores that sum to 1. This probability indicates the
     likelihood that the current image belongs to one of the 10 classes.
'''

model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

# Optimiser -> optimiser function used to update model and minimise loss
# Loss -> Measures how accurate the model is during training
# Metrics -> Monitors the training and testing steps. In this exmaple, accuracy is
#            monitored ie the fraction of images that are correctly classified.

model.fit(train_images, train_labels, epochs=5)
# Epochs refer to the feedforward and backpropagation cycles
print('Model fitting complete')

test_loss, test_acc = model.evaluate(test_images, test_labels)
print('Test accuracy:', test_acc)
# Evaluate the performance of the model against the test dataset

# The model reaches a training accuracy of about 89% but then a test accuracy of
# about 87%. The gap between the training and test accuracy is an example of
# over-fitting. Over-fitting => model performs worse on new data than training data

predictions = model.predict(test_images)
# Use the model after training it to predict what the images are

print('Confidence array:', predictions[0])
# Gives an array of confidences/(scaled) probabilities for what the image can be
# classified as

print('Most likely label: ', np.argmax(predictions[0]))
print('Actual label: ', test_labels[0])

num_rows = 5
num_cols = 3
num_images = num_rows*num_cols
print("Making figures...")

'''
i = 0
plt.figure(figsize=(6,3))
plt.subplot(1,2,1)
plot_image(i, predictions, test_labels, test_images)
plt.subplot(1,2,2)
plot_value_array(i, predictions,  test_labels)
'''

plt.figure(figsize=(2*2*num_cols, 2*num_rows))
for i in range(num_images):
    plt.subplot(num_rows, 2*num_cols, 2*i+1)
    plot_image(i, predictions, test_labels, test_images)
    plt.subplot(num_rows, 2*num_cols, 2*i+2)
    plot_value_array(i, predictions, test_labels)
plt.show()
# do not forget plt.show() or the graphs will not appear my g

img = test_images[0]
print(img.shape)

# arguments to expand_dims are (input_array, axis). The method adds a new axis
# at the axis specified. This is equivalent to adding the image to a batch where
# it is the only member.
img = (np.expand_dims(img, 0))
print(img.shape)

predictions_single = model.predict(img)
print(predictions_single)

plot_value_array(0, predictions_single, test_labels)
_ = plt.xticks(range(10), class_names, rotation=45)

np.argmax(predictions_single[0])




