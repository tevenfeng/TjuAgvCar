#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Dec 21 12:01:40 2017
Updated on April 14 2018

@author: GustavZ and AlexanderRobles21
"""
import numpy as np
import os
import six.moves.urllib as urllib
import tarfile
import tensorflow as tf
import cv2
import yaml
import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

# Protobuf Compilation (once necessary)
#os.system('protoc object_detection/protos/*.proto --python_out=.')

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from stuff.helper import FPS2, WebcamVideoStream

## LOAD CONFIG PARAMS ##
with open("/home/nvidia/AutonomousTju/src/ssd_mobilenet/config_jetson.yml", 'r') as ymlfile:
    cfg = yaml.load(ymlfile)
video_input = cfg['video_input']
visualize = cfg['visualize']
max_frames = cfg['max_frames']
width = cfg['width']
height = cfg['height']
fps_interval = cfg['fps_interval']
allow_memory_growth = cfg['allow_memory_growth']
det_interval = cfg['det_interval']
det_th = cfg['det_th']
model_name = cfg['model_name']
model_path = cfg['model_path']
label_path = cfg['label_path']
num_classes = cfg['num_classes']
  

class runSSDMobilenet(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.detection_pub = rospy.Publisher("/object_detection/image/compressed", CompressedImage)
        self.ssd()


    def ssd(self):                                
        self.download_model()    
        self.detection_graph, self.category_index = self.load_frozenmodel()

        # Start detection
        # Session Config: Limit GPU Memory Usage
        self.config = tf.ConfigProto()
        self.config.gpu_options.allow_growth=allow_memory_growth
    
        cur_frames = 0
        # Detection
      
        self.sess = tf.Session(graph=self.detection_graph, config = self.config)
        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        rospy.Subscriber("/usb_cam/image_raw", Image, self.raw_image_callback)


    def raw_image_callback(self, pic):
        image_np = self.bridge.imgmsg_to_cv2(pic)
        # Because image_np is read-only, copy it
        image_copy = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})
        # Visualization of the results of a detection.
        vis_util.visualize_boxes_and_labels_on_image_array(
        image_copy,
        np.squeeze(boxes),
        np.squeeze(classes).astype(np.int32),
        np.squeeze(scores),
        self.category_index,
        use_normalized_coordinates=True,
        line_thickness=8)
        # cv2.imshow('object_detection', image_np)
        # Exit Option
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpeg', image_copy)[1]).tostring()
        self.detection_pub.publish(msg)
        del image_copy


    def load_frozenmodel(self):
        # Load a (frozen) Tensorflow model into memory.
        detection_graph = tf.Graph()
        with detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(model_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
        # Loading label map
        label_map = label_map_util.load_labelmap(label_path)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=num_classes, use_display_name=True)
        category_index = label_map_util.create_category_index(categories)
        return detection_graph, category_index


    def download_model(self):
        model_file = model_name + '.tar.gz'
        download_base = 'http://download.tensorflow.org/models/object_detection/'   
        if not os.path.isfile(model_path):
            print('Model not found. Downloading it now.')
            opener = urllib.request.URLopener()
            opener.retrieve(download_base + model_file, model_file)
            tar_file = tarfile.open(model_file)
            for file in tar_file.getmembers():
              file_name = os.path.basename(file.name)
              if 'frozen_inference_graph.pb' in file_name:
                tar_file.extract(file, os.getcwd() + '/models/')
            os.remove(os.getcwd() + '/' + model_file)
        else:
            print('Model found. Proceed.')


if __name__ == '__main__':
    try:
        rospy.init_node('ssd_mobilenet_node', anonymous=True)
        runSSDMobilenet()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
