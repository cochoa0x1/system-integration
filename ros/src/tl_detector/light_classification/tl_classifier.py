from styx_msgs.msg import TrafficLight
#import matplotlib
#matplotlib.use('Agg')

#import matplotlib.pyplot as plt
import numpy as np

import cv2
import tensorflow as tf
import rospy
import traceback

import json

#SSD_GRAPH_FILE = 'light_classification/ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'
SSD_GRAPH_FILE = 'light_classification/faster_rcnn_resnet101_coco_11_06_2017/frozen_inference_graph.pb'


def load_graph(graph_file):
	"""Loads a frozen inference graph"""
	graph = tf.Graph()
	with graph.as_default():
		od_graph_def = tf.GraphDef()
		with tf.gfile.GFile(graph_file, 'rb') as fid:
			serialized_graph = fid.read()
			od_graph_def.ParseFromString(serialized_graph)
			tf.import_graph_def(od_graph_def, name='')
	return graph


def log(s):
	with open('logfile.txt','a') as f:
		f.write(s)


try:
	detection_graph = load_graph(SSD_GRAPH_FILE)
	with open('logfile.txt','wb') as f:
		f.write('LOADED GRAPH! <--------------------------------------------\n')


	detection_graph = load_graph(SSD_GRAPH_FILE)
	# detection_graph = load_graph(RFCN_GRAPH_FILE)
	# detection_graph = load_graph(FASTER_RCNN_GRAPH_FILE)

	# The input placeholder for the image.
	# `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
	image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

	# Each box represents a part of the image where a particular object was detected.
	detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

	# Each score represent how level of confidence for each of the objects.
	# Score is shown on the result image, together with the class label.
	detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')

	# The classification of the object (integer id).
	detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

	log('loaded all the other stuff')


except Exception as e:
	with open('logfile.txt','wb') as f:
		f.write('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n')
		f.write(traceback.format_exc())


log('tf version:')
log(str(tf.__version__))

def detect(image):

	log(str(type(image)))

	image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)
	with tf.Session(graph=detection_graph) as sess:                
		# Actual detection.
		(boxes, scores, classes) = sess.run([detection_boxes, detection_scores, detection_classes], feed_dict={image_tensor: image_np})

	#boxes = np.squeeze(boxes)
	#scores = np.squeeze(scores)
	#classes = np.squeeze(classes)

	log(json.dumps(boxes))
	log(json.dumps(scores))
	log(json.dumps(classes))


class TLClassifier(object):
	def __init__(self):
		log('classifier created')

	def get_classification(self, image):
		"""Determines the color of the traffic light in the image

		Args:
			image (cv::Mat): image containing the traffic light

		Returns:
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""
		#TODO implement light color prediction
		#cv2.imwrite('test.png',image) #works!
		try:
			detect(image)
		except Exception as e:
			log(traceback.format_exc())

		return TrafficLight.UNKNOWN
