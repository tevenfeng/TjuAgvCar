#!/usr/bin/python2

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import tensorflow as tf
from CNN.CnnModel import CnnModel
from cv_bridge import CvBridge, CvBridgeError
import cv2

from subprocess import call
from sensor_msgs.msg import Joy
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

"""
This node restores a saved TF model that was trained on a host computer
"""


class runCNN(object):

    def __init__(self):
        # ----------------------------
        self.sess = tf.InteractiveSession()
        self.model = CnnModel()
        saver = tf.train.Saver()
        saver.restore(self.sess, "/home/nvidia/AutonomousTju/data/models/model/model.ckpt")
        # ----------------------------
        self.startNavigationButton = rospy.get_param('start_navigation_button', 9)
        self.stopNavigationButton = rospy.get_param('stop_navigation_button', 8)
        self.bridge = CvBridge()
        self.netEnable = False
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.joy_pub = rospy.Publisher("/navigation", Joy, queue_size=1)
        self.img_pub = rospy.Publisher("/feature_map", Image, queue_size=10)
        rospy.init_node('car_cnn_node', anonymous=True)
        rospy.spin()

    def image_callback(self, pic):
        if self.netEnable:
            joy = Joy()
            cv2image = self.bridge.imgmsg_to_cv2(pic)
            cv2image = cv2.resize(cv2image, (320, 240), interpolation=cv2.INTER_CUBIC)
            cv2image = cv2image[50:170, :, :]
            normed_img = cv2image.astype(dtype=np.float32) / 255.0
            # normed_img = np.reshape(normed_img, (115, 200, 1))
            steer = self.model.y_out.eval(session=self.sess, feed_dict={self.model.x: [normed_img],
                                                                        self.model.keep_prob_fc1: 1.0,
                                                                        self.model.keep_prob_fc2: 1.0,
                                                                        self.model.keep_prob_fc3: 1.0,
                                                                        self.model.keep_prob_fc4: 1.0})

            #x = self.model.h_conv5.eval(session=self.sess, feed_dict={self.model.x: [normed_img],
            #                                                            self.model.keep_prob_fc1: 1.0,
            #                                                            self.model.keep_prob_fc2: 1.0,
            #                                                            self.model.keep_prob_fc3: 1.0,
            #                                                            self.model.keep_prob_fc4: 1.0})
            #x = x[0]
            #y = x[:,:,0]
            #for i in range(1,64):
            #    y += x[:,:,i]
            #try:
            #    img = self.bridge.cv2_to_imgmsg(y, "32FC1")
            #    self.img_pub.publish(img)
            #except CvBridgeError as e:
            #    print(e)

            if abs(steer[0][0]) < 0.0001:
                steer[0][0] = 0
            joy.axes.append(steer[0][0])
            joy.axes.append(1.0)			
            print("steering angle = %s\n" % str(joy.axes[0]))
            self.joy_pub.publish(joy)

    def joy_callback(self, joy):
        start_navigation_button_value = joy.buttons[self.startNavigationButton]
        if start_navigation_button_value and (not self.netEnable):
            self.netEnable = True
            print('Neural Network Enabled!\n')

        stop_navigation_button_value = joy.buttons[self.stopNavigationButton]
        if stop_navigation_button_value and self.netEnable:
            self.netEnable = False
            print('Neural Network Disabled!\n')


    def concat_features(self, conv_output):
        num_or_size_splits = conv_output.get_shape().as_list()[-1]
        each_convs = tf.split(conv_output, num_or_size_splits=num_or_size_splits, axis=3)
        concact_size = int(math.sqrt(num_or_size_splits) / 1)
        all_concact = None
        for i in range(concact_size):
            row_concact = each_convs[i * concact_size]
            for j in range(concact_size - 1):
                row_concact = tf.concat([row_concact, each_convs[i * concact_size + j + 1]], 1)
            if i == 0:
                all_concact = row_concact
            else:
                all_concact = tf.concat([all_concact, row_concact], 2)

        return all_concact


if __name__ == '__main__':
    try:
        runCNN()
    except rospy.ROSInterruptException:
        pass
