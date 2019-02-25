#!/usr/bin/python2

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf


def weight_variable(shape):
    '''Initialize weight variable with specified shape.
    shape: [kernel_shape[0], kernel_shape[1], num_of_channels, num_of_kernels]'''
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)


def bias_variable(shape):
    '''Initialize bias variable with specified shape.
    shape: number of bias, equals to the number of kernels in the same layer.'''
    initial = tf.constant(0.0, shape=shape)
    return tf.Variable(initial)


def conv2d(x, W, stride):
    '''Get 2d convolutional layer with given parameters.
    x: input of the layer
    W: weight tensor
    stride: single number determining the stride
    padding: we set default padding to \'SAME\''''
    return tf.nn.conv2d(x, W, strides=[1, stride, stride, 1], padding='SAME')


def conv_layer(x, kernel_shape=(3, 3), stride=1, num_of_kernels=32, use_bias=False):
    '''Function to form a whole group of convolutional layer including
    a conv2d layer and a relu layer(PS: currently max pooling layer is ignored).'''
    W = weight_variable([kernel_shape[0], kernel_shape[1], x.get_shape()[-1].value, num_of_kernels])
    if use_bias:
        b = bias_variable([num_of_kernels])
        return tf.nn.relu(conv2d(x, W, stride=stride) + b), W
    else:
        return tf.nn.relu(conv2d(x, W, stride=stride)), W


def fc_layer(x, num_of_neurons, activation=tf.tanh, use_bias=True, dropout=False):
    '''Function to form a fully connected layer with the given parameters.
    x: input of this fully connected layer
    num_of_neurons: number of neurons included in this layer
    activation: activation function type
    '''
    W = weight_variable([x.get_shape()[-1].value, num_of_neurons])
    h, b = None, None
    if use_bias:
        b = bias_variable([num_of_neurons])
        h = activation(tf.matmul(x, W) + b)
    else:
        h = activation(tf.matmul(x, W))

    if dropout:
        keep_prob = tf.placeholder(tf.float32)
        h_drop = tf.nn.dropout(h, keep_prob)
        return h_drop, W, b, keep_prob
    else:
        return h, W, b, None


def flattened(x):
    product = 1
    for d in x.get_shape():
        if d.value is not None:
            product *= d.value
    return tf.reshape(x, [-1, product])
