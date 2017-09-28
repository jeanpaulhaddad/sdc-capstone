from styx_msgs.msg import TrafficLight
import cv2
from keras.models import load_model
from numpy import zeros, newaxis

class TLClassifier(object):
    def __init__(self):
        model = load_model('test_1.h5')
        print(model)
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        imrs = imrs.astype(float)
    	imrs = imrs / 255.0

    	imrs = imrs[newaxis,:,:,:]
        print(imrs)
    	preds = model.predict(imrs)
    	print('Predicted:' ,preds)
    	predicted_class = np.argmax(preds, axis=1)

    	print('Predicted C:' ,predicted_class)
    	lid = predicted_class[0]

        if(lid == 1):
           return TrafficLight.RED

        return TrafficLight.UNKNOWN
