#!/usr/bin/python
# coding:utf8

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


import tensorflow as tf
import numpy as np
from CnnModel import CnnModel

import cv2 as cv
import os


def load_dataset(path, percent_testing=None):
    assert percent_testing is None or (0.0 <= percent_testing <= 1.0)
    x, y, fnames = [], [], []
    for root, sub_dirs, files_ in os.walk(path):
        for name in files_:
            print(name)
            if name == '.DS_Store':
                continue
            img = cv.imread(os.path.join(root, name))
            img = cv.resize(img, (200, 150), interpolation=cv.INTER_CUBIC)
            img = img[:115, :, :]  # ?
            x.append(img)


            # filename format: seq_linear_angular_stamp.sec_stamp.nsec
            # _, linear, angular, sec, nsec = fname.split('_')
            # linear, angular, sec, nsec = float(linear), float(angular), int(sec), int(nsec.split('.png')[0])
            sec, nsec, linear, angular = name.split('_')
            sec, nsec, linear, angular = int(sec), int(nsec), float(linear), float(angular.split('.png')[0])
            print('[linear: %f, angular: %f, sec: %d, nsec: %d]'%(linear, angular, sec, nsec))
            y.append((linear, angular))

    train_x, train_y, test_x, test_y = [], [], [], []
    if percent_testing is not None:
        tst_strt = int(len(x) * (1.0 - percent_testing))
        train_x, train_y, test_x, test_y = x[:tst_strt], y[:tst_strt], x[tst_strt:], y[tst_strt:]
    else:
        train_x, train_y, test_x, test_y = x, y, x, y
    return train_x, train_y, test_x, test_y


if __name__ == '__main__':
    path = r'/Users/tevenfeng/Coding/python/backup/data/4floor/'
    train_x, train_y, test_x, test_y = load_dataset(path=path)

    num_epochs = 100
    batch_size = 100

    # Drop items from dataset so that it's divisible by batch_size
    train_x = train_x[0:-1 * (len(train_x) % batch_size)]
    train_y = train_y[0:-1 * (len(train_y) % batch_size)]
    test_x = test_x[0:-1 * (len(test_x) % batch_size)]
    test_y = test_y[0:-1 * (len(test_y) % batch_size)]

    print('len(test_x) =', len(test_x))

    batches_per_epoch = int(len(train_x) / batch_size)

    print('len(test_x) =', len(train_x))

    sess = tf.InteractiveSession()
    model = CnnModel()
    train_step = tf.train.AdamOptimizer(1e-4).minimize(model.loss)
    saver = tf.train.Saver()
    sess.run(tf.initialize_all_variables())

    for i in range(num_epochs):

        for b in range(0, batches_per_epoch):
            batch = [train_x[b * batch_size:b * batch_size + batch_size],
                     train_y[b * batch_size:b * batch_size + batch_size]]
            # --- normalize batch ---
            batch_ = [[], []]
            for j in range(len(batch[0])):
                batch_[0].append(batch[0][j].astype(dtype=np.float32) / 255.0)
                batch_[1].append(np.array([batch[1][j][1]], dtype=np.float32))
            batch = batch_
            # ------------------------
            train_step.run(
                feed_dict={model.x: batch[0], model.y_: batch[1], model.keep_prob_fc1: 0.8, model.keep_prob_fc2: 0.8,
                           model.keep_prob_fc3: 0.8, model.keep_prob_fc4: 0.8})
            # train_error = model.loss.eval(feed_dict={model.x:batch[0], model.y_:batch[1],
            #                                         model.keep_prob_fc1:1.0, model.keep_prob_fc2:1.0,
            #                                         model.keep_prob_fc3:1.0, model.keep_prob_fc4:1.0})
            # print("epoch %d, training entropy %g"%(i, train_error))
        print('epoch', i, 'complete')
        if i % 5 == 0:
            test_error = 0.0
            for b in range(0, len(test_x), batch_size):
                batch = [test_x[b:b + batch_size], test_y[b:b + batch_size]]
                # --- normalize batch ---
                batch_ = [[], []]
                for j in range(len(batch[0])):
                    batch_[0].append(batch[0][j].astype(dtype=np.float32) / 255.0)
                    batch_[1].append(np.array([batch[1][j][1]], dtype=np.float32))
                batch = batch_
                # print('batch =', len(batch[0]), len(batch[1]))
                test_error_ = model.loss.eval(feed_dict={model.x: batch[0], model.y_: batch[1],
                                                         model.keep_prob_fc1: 1.0, model.keep_prob_fc2: 1.0,
                                                         model.keep_prob_fc3: 1.0, model.keep_prob_fc4: 1.0})
                # -----------------------
                test_error += test_error_
            test_error /= len(test_x) / batch_size
            test_accuracy = 1.0 - test_error
            print("test accuracy %g" % test_accuracy)
    filename = saver.save(sess, r'/Users/tevenfeng/Coding/python/TrainCNN/model/model.ckpt')
