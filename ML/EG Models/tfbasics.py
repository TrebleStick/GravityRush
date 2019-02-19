import tensorflow as tf

# NB: To run stuff from cmd prompt use python, not python3

x1 = tf.constant(5)
x2 = tf.constant(6)
# x1 and x2 are constants - 5 and 6 respectively

# result = x1 * x2
result = tf.multiply(x1, x2)
print(result)
# The result of this operation is a tensor. It does not have any value because nothing
# has actually been run. Appears to work even if you use x1 * x2

sess = tf.Session()
print(sess.run(result))
sess.close()
# Prints the result of the calculation. The actual calculation did not happen until we
# ran the actual session though!

# This with... as clause will run the session and then close it once done -> convenience
with tf.Session() as sess:
    output = sess.run(result)
    print(output)
    # output becomes a python variable and can be accessed from outside this clause

print(output)
# will not give an error
print(sess.run(result))
# will give an error - session is closed

