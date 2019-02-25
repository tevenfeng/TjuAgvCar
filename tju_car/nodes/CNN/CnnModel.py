#!/usr/bin/python2

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf
from CNN.TensorflowUtil import weight_variable, bias_variable
from CNN.TensorflowUtil import conv_layer, fc_layer, flattened


class CnnModel(object):
    '''Car CNN model'''

    def __init__(self):
        self.x = tf.placeholder(tf.float32, [None, 50, 200, 3])
        self.y_ = tf.placeholder(tf.float32, [None, 1])

        (self.h_conv1, _) = conv_layer(self.x, kernel_shape=(5, 5), stride=2, num_of_kernels=24, use_bias=True)
        (self.h_conv2, _) = conv_layer(self.h_conv1, kernel_shape=(5, 5), stride=2, num_of_kernels=36, use_bias=True)
        (self.h_conv3, _) = conv_layer(self.h_conv2, kernel_shape=(5, 5), stride=2, num_of_kernels=48, use_bias=True)
        (self.h_conv4, _) = conv_layer(self.h_conv3, kernel_shape=(3, 3), stride=1, num_of_kernels=64, use_bias=True)
        (self.h_conv5, _) = conv_layer(self.h_conv4, kernel_shape=(3, 3), stride=1, num_of_kernels=64, use_bias=True)

        self.h_conv5_flat = flattened(self.h_conv5)

        (self.h_fc1_drop, _, _, self.keep_prob_fc1) = fc_layer(x=self.h_conv5_flat, num_of_neurons=512,
                                                               activation=tf.nn.relu, use_bias=True, dropout=True)
        (self.h_fc2_drop, _, _, self.keep_prob_fc2) = fc_layer(self.h_fc1_drop, 100, tf.nn.relu, True, True)
        (self.h_fc3_drop, _, _, self.keep_prob_fc3) = fc_layer(self.h_fc2_drop, 50, tf.nn.relu, True, True)
        (self.h_fc4_drop, _, _, self.keep_prob_fc4) = fc_layer(self.h_fc3_drop, 10, tf.nn.relu, True, True)
        W_fc5 = weight_variable([10, 1])
        b_fc5 = bias_variable([1])

        self.y_out = tf.matmul(self.h_fc4_drop, W_fc5) + b_fc5
        self.loss = tf.reduce_mean(tf.abs(tf.subtract(self.y_, self.y_out)))
