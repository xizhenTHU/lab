import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split
import numpy as np  # linear algebra
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

data = pd.read_csv('train.csv')
label = data.label

data_35 = data[(label == 3) | (label == 5)]
X_train = data_35.drop(labels=["label"], axis=1)
Y_train = data_35["label"]
Y_train[Y_train == 3] = 1
Y_train[Y_train == 5] = -1
del data
del label

random_seed = 2
X_train, X_test, Y_train, Y_test = train_test_split(
    X_train, Y_train, test_size=0.2, random_state=random_seed)
X_train.index = [i for i in range(X_train.shape[0])]
Y_train.index = [i for i in range(Y_train.shape[0])]


batch_size = 128
# 初始化feedin
x_data = tf.placeholder(shape=[None, 784], dtype=tf.float32)
y_target = tf.placeholder(shape=[None, 1], dtype=tf.float32)

# 创建权值参数
W = tf.Variable(tf.truncated_normal(shape=[784, 1]))
b = tf.Variable(tf.truncated_normal(shape=[1, 1]))


# 定义线性模型: y = Ax + b
model_output = tf.add(tf.matmul(x_data, W), b)

# Declare vector L2 'norm' function squared
l2_norm = tf.reduce_sum(tf.square(W))

# Loss = max(0, 1-pred*actual) + alpha * L2_norm(A)^2
alpha = tf.constant([0.01])
classification_term = tf.reduce_mean(tf.maximum(
    0., tf.subtract(1., tf.multiply(model_output, y_target))))
# classification_term = tf.reduce_sum(tf.maximum(
#     tf.zeros(batch_size, 1), tf.subtract(1., tf.multiply(model_output, y_target))))
loss = tf.add(classification_term, tf.multiply(alpha, l2_norm))


# gamma = tf.constant(-10.0)
# dist = tf.reduce_sum(tf.square(tf.transpose(x_data)), 1)
# dist = tf.reshape(dist, [-1,1])
# sq_dists = tf.add(tf.subtract(dist, tf.multiply(2., tf.matmul(tf.transpose(x_data),x_data))), tf.transpose(dist))
# my_kernel = tf.exp(tf.multiply(gamma, sq_dists))
# output = tf.add(tf.matmul(tf.transpose(W),my_kernel),b)

# test
X_place = tf.placeholder(tf.float32,[2,None])
Y_place = tf.placeholder(tf.float32,[1,None])
w = tf.Variable(tf.random_normal(shape=[1,X_train.shape[1]]))
b = tf.Variable(tf.random_normal(shape=[1,1]))
#compute Gaussian Kernel 
gamma = tf.constant(-10.0)
dist = tf.reduce_sum(tf.square(tf.transpose(X_place)), 1)
dist = tf.reshape(dist, [-1,1])
sq_dists = tf.add(tf.subtract(dist, tf.multiply(2., tf.matmul(tf.transpose(X_place),X_place))), tf.transpose(dist))
my_kernel = tf.exp(tf.multiply(gamma, sq_dists))
output = tf.add(tf.matmul(w,my_kernel),b)
loss2 = tf.reduce_mean(tf.maximum(0.,tf.subtract(1.,tf.multiply(output,Y_place)))) + tf.matmul(w,tf.transpose(w))


prediction = tf.sign(model_output)
predictions_correct = tf.cast(tf.equal(prediction, y_target), tf.float32)
accuracy = tf.reduce_mean(predictions_correct)

my_opt = tf.train.GradientDescentOptimizer(0.001)
train_step = my_opt.minimize(loss)

with tf.Session() as sess:
    init = tf.global_variables_initializer()
    sess.run(init)

# Training loop
    loss_vec = []
    train_acc = []
    test_acc = []
    for i in range(2000):
        rand_index = np.random.choice(len(X_train), size=batch_size)
        rand_x = X_train.loc[rand_index, :]
        rand_y = np.transpose([Y_train[rand_index]])
        sess.run(train_step, feed_dict={x_data: rand_x, y_target: rand_y})
        temp_loss = sess.run(
            loss, feed_dict={x_data: rand_x, y_target: rand_y})
        loss_vec.append(temp_loss)
        temp_acc_train = sess.run(
            accuracy,
            feed_dict={
                x_data: X_train,
                y_target: Y_train[:, np.newaxis]})
        train_acc.append(temp_acc_train)
        temp_acc_test = sess.run(
            accuracy, feed_dict={
                x_data: X_test, y_target: Y_test[:, np.newaxis]})
        test_acc.append(temp_acc_test)
        if (i + 1) % 100 == 0:
            print('Loss = ' + str(temp_loss))
        temp_sq_dists=sess.run(sq_dists, feed_dict={x_data: rand_x, y_target: rand_y})
        print(temp_sq_dists)

    plt.ion()
    plt.figure(1)
    plt.plot(loss_vec, 'k-')
    plt.title('Loss per Generation')
    plt.xlabel('Generation')
    plt.ylabel('Loss')

    plt.figure(2)
    plt.plot(train_acc, 'k-', label='Train Set Accuracy')
    plt.plot(test_acc, 'r--', label='Test Set Accuracy')
    plt.title('Train and Test Accuracy')
    plt.xlabel('Generation')
    plt.ylabel('Accuracy')
    plt.legend(loc='lower right')
    plt.ioff()
    plt.show()


