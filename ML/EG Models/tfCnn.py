from __future__ import absolute_import, division, print_function
# Comment out the above line and see what it affects

import tensorflow as tf
import numpy as np

'''
Convolutional neural networks consist of the following:
- Convolutional Layers: Apply a specified number of convolutional filters to the image.
  For each subregion, the layer performs a set of mathematical operations to produce a
  single value in the output feature map. These layers then typically apply a ReLU activation
  function to the output to introduce non-linearities to the model
- Pooling Layers: Downsample the image data extracted by the convolutional layers to reduce
  the dimensionality of the feature map. A common pooling algorithm is max. pooling -> extracts
  subregions of the feature map, keeps their maximum value and discards all other values.
- Densely/Fully Connected Layers: Classifies based on features extracted by convolutional layers
  and downsampled in the pooling layers.

A typical CNN is composed of a stack of convolutional modules that perform feature extraction.
Each module consists of a convolutional layer and a pooling layer. The last convolutional module
is followed by one or more dense layers to perform classification.

The model below classifies images in the MNIST dataset using the following CNN architecture:
1) Convolutional Layer 1: Applies 32 5x5 filters (extracting 5x5 pixel subregions) with ReLU
   activation function.
2) Pooling Layer 1: Performs max pooling with a 2x2 filter and a stride of 2, which means that
   pooled regions should not overlap.
3) Convolutional Layer 2: Applies 64 5x5 filters with ReLU activation
4) Pooling Layer 2: Same as Pooling Layer 1
5) Dense Layer 1: 1,024 neurons with dropout regularisation rate of 0.4.
   - 'Dropout' is a technique often used in deep learning where some neurons are picked at random
   and completely ignored for that epoch. This forces surrounding neurons to take build their own
   representation of the dataset. This technique essentially limits overfitting.
6) Dense Layer 2: 10 neurons - one for each target class (0-9). We are trying to classify images
   of handwritten digits into one of these ten classes
'''

def cnn_model_fn(features, labels, mode):
    # Input layer - the methods for creating convolutional and pooling layers for 2D images expect
    #               input tensors to have shape [batch_size, image_height, image_width, channels] by
    #               default.
    #               Our MNIST dataset is composed of monochrome 28x28pixel images so the desired shape
    #               for the input layer is: [batch_size, 28, 28, 1]. -1 specifies that this dimension
    #               should be dynamically computed based on the number of input values in features["x"].
    #               This means that batch_size can be tuned. If we feed examples into our model in batches
    #               of 5, features["x"] will contain 3,920 (784 values from 5 images).
    # NB - batch_size refers to the size of the subset of examples to use when performing gradient
    #      descent during training
    #    - channels refers to number of colour channels in the example images. Colour images have RGB
    #      so 3 channels. Monochrome images have black so 1 channel.
    input_layer = tf.reshape(features["x"], [-1, 28, 28, 1])

    # Convolutional Layer 1
    conv1 = tf.layers.conv2d(
        inputs=input_layer,
        filters=32,
        # conv1 has shape [batch_size, 28, 28, 32] -> 32 channels due to output from each filter
        kernel_size=[5, 5],
        padding="same",
        # padding specifies one of two enumerated values - valid or same. Same specifies that the output tensor should
        # have the same height and width values as the input tensor. In this case, a 5x5 convolution over a 28x28 tensor
        # produces a 24x24 tensor so 0-padding is added to make up the necessary height
        activation=tf.nn.relu
    )

    # Pooling Layer 1
    # pool1 has shape[batch_size, 14, 14, 32] -> the 2x2 filter reduces the height and width by 50%
    pool1 = tf.layers.max_pooling2d(inputs=conv1, pool_size=[2, 2], strides=2)

    # Convolutional Layer 2
    # conv2 has the shape [batch_size, 14, 14, 64] -> same height and width as pool1 due to padding="same"
    conv2 = tf.layers.conv2d(
        inputs=pool1,
        filters=64,
        kernel_size=[5, 5],
        padding="same",
        activation=tf.nn.relu
    )

    # Pooling Layer 2
    # pool2 has shape [batch_size, 7, 7, 64]
    pool2 = tf.layers.max_pooling2d(inputs=conv2, pool_size=[2, 2], strides=2)

    # Dense Layer
    # Feature map needs to be flattens to [batch_size, features] so out tensor has only 2 dimensions
    # Each example has 7*7*63=3136 features
    pool2_flat = tf.reshape(pool2, [-1, 7*7*64])
    dense = tf.layers.dense(inputs=pool2_flat, units=1024, activation=tf.nn.relu)
    dropout = tf.layers.dropout(inputs=dense,
                                rate=0.4,
                                training=mode == tf.estimator.ModeKeys.TRAIN)

    # Logits Layer
    logits = tf.layers.dense(inputs=dropout,
                             units=10)

    predictions = {
        # Generate predictions (for PREDICT and EVAL mode)
        # The below just gives you the value of the predicted class (0-9)
        "classes": tf.argmax(input=logits,
                             axis=1),
        # Add 'softmax_tensor' to the graph. It is used for PREDICT and by the 'logging_hook'
        # The below gives you whole list of probabilities at the output for that image/exmaple
        # axis specifies which axis of the input tensor we want to apply the operation to
        "probabilities": tf.nn.softmax(logits, name="softmax_tensor")
    }

    if mode == tf.estimator.ModeKeys.PREDICT:
        return tf.estimator.EstimatorSpec(mode=mode, predictions=predictions)

    # Calculate Loss (for both TRAIN and EVAL modes)
    loss = tf.losses.sparse_softmax_cross_entropy(labels=labels,
                                                  logits=logits)

    if mode == tf.estimator.ModeKeys.TRAIN:
        optimiser = tf.train.GradientDescentOptimizer(learning_rate=0.001)
        train_op = optimiser.minimize(
            loss=loss,
            global_step=tf.train.get_global_step()
        )
        return tf.estimator.EstimatorSpec(mode=mode, loss=loss, train_op=train_op)

    # Add evaluation metrics (for EVAL mode)
    eval_metric_ops = {
        "accuracy": tf.metrics.accuracy(labels=labels,
                                        predictions=predictions["classes"])
    }
    return tf.estimator.EstimatorSpec(mode=mode, loss=loss, eval_metric_ops=eval_metric_ops)

((train_data, train_labels),
 (eval_data, eval_labels)) = tf.keras.datasets.mnist.load_data()

train_data /= np.float32(255) # normalise pixel values
train_labels = train_labels.astype(np.int32) # cast as an integer... not necessary

eval_data /= np.float32(255)
eval_labels = eval_labels.astype(np.int32)

# Create the Estimator
mnist_classifier = tf.estimator.Estimator(
    model_fn=cnn_model_fn,
    # the below specifies where model data will be saved
    model_dir="/tmp/mnist_convnet_model"
)










