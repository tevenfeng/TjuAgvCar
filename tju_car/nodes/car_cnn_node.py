#!/usr/bin/python3
# coding:utf8

import rospy
import tensorflow as tf
import numpy as np
import cv2 as cv
import cv_bridge


class car_cnn_node(object):
	def __init__(self):
		print("Car CNN node init.\n")
		rospy.init_node('car_cnn_node', anonymous=True)
		rospy.spin()


if __name__ == '__main__':
	try:
		car_cnn_node()
	except rospy.ROSInterruptException:
		print("Error!\n")
		pass
