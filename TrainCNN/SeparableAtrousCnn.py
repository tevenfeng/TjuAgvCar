#!/usr/bin/python2
# coding:utf8

import tensorflow as tf


class SeparableAtrousCnn(object):
    '''
    ==================================================================
    Based on Nvidia's Paper with depthwise separable convolution and
    atrous convolution used.
    ==================================================================
    '''

    def __init__(self):
        self.x = tf.placeholder(tf.float32, [None, 120, 320, 3])
        self.y_ = tf.placeholder(tf.float32, [None, 1])
        self.keep_prob_fc1 = tf.placeholder(tf.float32)
        self.keep_prob_fc2 = tf.placeholder(tf.float32)
        self.keep_prob_fc3 = tf.placeholder(tf.float32)
        self.keep_prob_fc4 = tf.placeholder(tf.float32)

        self.conv1 = tf.layers.SeparableConv2D(filters=24, kernel_size=3, strides=1, padding='same',
                                               dilation_rate=(2, 2),
                                               activation=tf.nn.relu, use_bias=True,
                                               bias_initializer=tf.initializers.truncated_normal(0, 0.1))(self.x)

        self.conv2 = tf.layers.SeparableConv2D(filters=24, kernel_size=3, strides=2, padding="same",
                                               activation=tf.nn.relu, use_bias=True,
                                               bias_initializer=tf.initializers.truncated_normal(0, 0.1))(
            self.conv1)

        self.conv3 = tf.layers.SeparableConv2D(filters=36, kernel_size=3, strides=1, padding="same",
                                               dilation_rate=(2, 2),
                                               activation=tf.nn.relu, use_bias=True,
                                               bias_initializer=tf.initializers.truncated_normal(0, 0.1))(
            self.conv2)

        self.conv4 = tf.layers.SeparableConv2D(filters=36, kernel_size=3, strides=2, padding="same",
                                               activation=tf.nn.relu, use_bias=True,
                                               bias_initializer=tf.initializers.truncated_normal(0, 0.1))(
            self.conv3)

        self.conv5 = tf.layers.SeparableConv2D(filters=48, kernel_size=3, strides=1, padding="same",
                                               dilation_rate=(2, 2),
                                               activation=tf.nn.relu, use_bias=True,
                                               bias_initializer=tf.initializers.truncated_normal(0, 0.1))(
            self.conv4)

        self.conv6 = tf.layers.SeparableConv2D(filters=48, kernel_size=3, strides=2, padding="same",
                                               activation=tf.nn.relu, use_bias=True,
                                               bias_initializer=tf.initializers.truncated_normal(0, 0.1))(
            self.conv5)

        self.conv7 = tf.layers.SeparableConv2D(filters=64, kernel_size=3, strides=2, padding="same",
                                               activation=tf.nn.relu, use_bias=True,
                                               bias_initializer=tf.initializers.truncated_normal(0, 0.1))(
            self.conv6)

        self.conv8 = tf.layers.SeparableConv2D(filters=64, kernel_size=3, strides=2, padding="same",
                                               activation=tf.nn.relu, use_bias=True,
                                               bias_initializer=tf.initializers.truncated_normal(0, 0.1))(
            self.conv7)

        self.flatten = tf.layers.Flatten()(self.conv8)

        self.fc1 = tf.layers.Dense(units=512, activation=tf.nn.relu, use_bias=True,
                                   bias_initializer=tf.initializers.truncated_normal(0, 0.1))(self.flatten)
        self.dropout1 = tf.layers.Dropout(self.keep_prob_fc1)(self.fc1)

        self.fc2 = tf.layers.Dense(units=100, activation=tf.nn.relu, use_bias=True,
                                   bias_initializer=tf.initializers.truncated_normal(0, 0.1))(self.dropout1)
        self.dropout2 = tf.layers.Dropout(self.keep_prob_fc2)(self.fc2)

        self.fc3 = tf.layers.Dense(units=50, activation=tf.nn.relu, use_bias=True,
                                   bias_initializer=tf.initializers.truncated_normal(0, 0.1))(self.dropout2)
        self.dropout3 = tf.layers.Dropout(self.keep_prob_fc3)(self.fc3)

        self.fc4 = tf.layers.Dense(units=10, activation=tf.nn.relu, use_bias=True,
                                   bias_initializer=tf.initializers.truncated_normal(0, 0.1))(self.dropout3)
        self.dropout4 = tf.layers.Dropout(self.keep_prob_fc4)(self.fc4)

        self.y_out = tf.layers.Dense(units=1, activation=None, use_bias=True,
                                     bias_initializer=tf.initializers.truncated_normal(0, 0.1))(self.dropout4)

        self.loss = tf.reduce_mean(tf.abs(tf.subtract(self.y_, self.y_out)))
