'''
Using MNIST data set
    Data set is a series of 28x28 pixel black and white images of handwritten numbers from 0 to 9.
    60,000 images are used for training and 10,000 are used to actually test the model.

    Sequence of events in the network:
    # Input Data -> Weight Inputs -> Hidden Layer 1 (activation function) -> Weights
    # -> Hidden layer 2 -> Weights -> Output Layer
        - Data moves forward through the network here so this is the feedforward section
    # Compare output to intended output (cost function)
        - Cross entropy error function is an example of a cost function
    # Attempt to minimise cost function using an optimiser (optimisation function)
        - EG: Adam Optimiser, Stochastic Gradient Descent, AdaGrad etc.
        - The optimiser returns to original network and modifies weights to minimise cost. This is
          back-propagation
    # One iteration of feedforward and back-propagation is an epoch. You run this process over
      several epochs and observe the cost function decrease (hopefully)
'''

import tensorflow as tf

from tensorflow.examples.tutorials.mnist import input_data

mnist = input_data.read_data_sets("/tmp/data", one_hot=True)

'''
The final output will be one of 10 classes: 0-9
Using the one_hot parameter, the output will only switch on one output as below:

0 = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
1 = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
2 = [0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
etc...
'''

# Each layer has 500 nodes rn but feel free to alternate this. The number of nodes in
# each layer does not have to be the same. Everything depends on your model.
n_nodes_hl1 = 500
n_nodes_hl2 = 500
n_nodes_hl3 = 500

n_classes = 10
# Performs epochs of the images in batches of 100 at a time rather than all at once
batch_size = 100

# [height, width] - pixel data can be flattened to a 1D array of length 784 (28x28)
x = tf.placeholder('float', [None, 784])
y = tf.placeholder('float')

# TODO: Create the layers using a for loop

def neural_network_model(data):
    # The below is a key-value pair (dict.). The value of weights is a tensor flow
    # variable (tf.Variable) that consists of a tensor that (I think) consists of
    # random weights used to start the learning process. In layer 1 for example,
    # there are 784 inputs and n_nodes_hl1 that they each connect to. Each connection
    # needs a weight and so there are 784*n_nodes_hl1 weights.

    # (input_data * weights) + biases - biases prevent high occurrence of a zero
    # output. This means that neurons can still fire even if all inputs are zero.

    hidden_1_layer = {'weights': tf.Variable(tf.random_normal([784, n_nodes_hl1])),
                      'biases': tf.Variable(tf.random_normal([n_nodes_hl1]))}
    # All arguments to tf.random.normal need [] around them

    hidden_2_layer = {'weights': tf.Variable(tf.random_normal([n_nodes_hl1, n_nodes_hl2])),
                      'biases': tf.Variable(tf.random_normal([n_nodes_hl2]))}

    hidden_3_layer = {'weights': tf.Variable(tf.random_normal([n_nodes_hl2, n_nodes_hl3])),
                      'biases': tf.Variable(tf.random_normal([n_nodes_hl3]))}

    output_layer = {'weights': tf.Variable(tf.random_normal([n_nodes_hl3, n_classes])),
                    'biases': tf.Variable(tf.random_normal([n_classes]))}

    # (input_data * wieghts) + biases
    l1 = tf.add(tf.matmul(data, hidden_1_layer['weights']), hidden_1_layer['biases'])
    # The above is our summing operation on all the weighted inputs
    l1 = tf.nn.relu(l1)
    # The above is the threshold (activation) function (rectified linear) for sum

    l2 = tf.add(tf.matmul(l1, hidden_2_layer['weights']), hidden_2_layer['biases'])
    l2 = tf.nn.relu(l2)

    l3 = tf.add(tf.matmul(l2, hidden_3_layer['weights']), hidden_3_layer['biases'])
    l3 = tf.nn.relu(l3)

    output = tf.matmul(l3, output_layer['weights']) + output_layer['biases']
    # Output is not rectified. It is a one hot array

    return output

def train_neural_network(x):
    prediction = neural_network_model(x)
    cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(prediction, y))

    # AdamOptimiser function can take a float value argument that defines learning rate.
    # Default rate = 0.001 - which is fine for now. Optimiser is used to minimise cost
    # between prediction from neural net and y (the actual value).
    optimiser = tf.train.AdamOptimizer().minimize(cost)

    n_epochs = 10

    with tf.Session as sess:
        sess.run(tf.initialize_all_variables())

        for epoch in range(n_epochs):
            epoch_loss = 0
            # _ here is shorthand for all values
            for _ in range(int(mnist.train.num_examples/batch_size)):
                epoch_x, epoch_y = mnist.train.next_batch(batch_size)
                _, c = sess.run([optimiser, cost], feed_dict={x: epoch_x, y: epoch_y})
                epoch_loss += c
            print('Epoch', epoch, 'completed out of', n_epochs, 'loss: ', epoch_loss)

        correct = tf.equal(tf.argmax(prediction, 1), tf.argmax(y, 1))
        accuracy = tf.reduce_mean(tf.cast(correct, 'float'))
        print('Accuracy: ', accuracy.eval({x: mnist.test.images, y:mnist.test.labels}))

train_neural_network(x)



