from styx_msgs.msg import TrafficLight
import matplotlib.pyplot as plt

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        
        fig = plt.figure()
        data = np.zeros((nx, ny))
        self.im = plt.imshow(data, cmap='gist_gray_r', vmin=0, vmax=1)
        plt.show()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
