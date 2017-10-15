from styx_msgs.msg import TrafficLight
#import matplotlib
#matplotlib.use('Agg')

#import matplotlib.pyplot as plt
#import numpy as np

#import cv2
#import tensorflow as tf

class TLClassifier(object):
	def __init__(self):
		#TODO load classifier
		pass
		
		

	def get_classification(self, image):
		"""Determines the color of the traffic light in the image

		Args:
			image (cv::Mat): image containing the traffic light

		Returns:
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""
		#TODO implement light color prediction
		#plt.plot(np.random.randn(1000).cumsum())
		#plt.imshow(image)
		#plt.savefig('test.png')
		#plt.clf()

		#cv2.imwrite('test.png',image) #works!

		return TrafficLight.UNKNOWN
