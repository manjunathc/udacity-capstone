from styx_msgs.msg import TrafficLight
import numpy as np
import os
import sys
import tarfile
import tensorflow as tf
from PIL import Image
import rospy

def getTensorData(self):
    self.detection_graph = tf.Graph()
    with self.detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')
    with tf.Session(graph=self.detection_graph) as sess:
        # Definite input and output Tensors for detection_graph
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
    return(image_tensor,detection_boxes,detection_scores,detection_classes,num_detections)


# In[3]:


def load_image_into_numpy_array(image):
  #print(image)
  (im_width, im_height) = image.shape[:2]
  #rospy.logwarn("(im_width, im_height) 	-->",(im_width, im_height))
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)


# In[4]:


def detectImage(self,image):
    with tf.Session(graph=self.detection_graph) as sess:
        #image = Image.open(image_path)
        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        #image_np = load_image_into_numpy_array(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)
        # Actual detection.
        (scores, classes) = sess.run(
        [self.detection_scores, self.detection_classes],
        feed_dict={self.image_tensor: image_np_expanded})
        return (scores, classes)

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # What model to download.
        self.MODEL_NAME = 'trafic_light'
        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        self.PATH_TO_CKPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), self.MODEL_NAME , 'frozen_inference_graph.pb')
        # List of the strings that is used to add correct label for each box.
        self.PATH_TO_LABELS = os.path.join('data', 'object-detection.pbtxt')
        self.NUM_CLASSES = 3
        self.detection_graph = tf.Graph()
        self.min_score_thresh=.5
        (self.image_tensor,self.detection_boxes,self.detection_scores,self.detection_classes,self.num_detections) = getTensorData(self)
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
        image (cv::Mat): image containing the traffic light
        Returns:
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction
        #image_np = load_image_into_numpy_array(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        #image_np_expanded = np.expand_dims(image, axis=0)
        
        (scores, classes) = detectImage(self,image)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        id_class = 1
        max_scores = 3   
        
        for i in range(max(0,max_scores)):
            if scores[i] > self.min_score_thresh:
                #print("> Thresh - classes-->",classes[i])
                #print("> Thresh - score-->",scores[i])
		id_class = classes[i]
                if (i > 0 and classes[i] != 1):
                    id_class = 3
		else:
		    id_class = classes[i]

	#print("id_class after for-->",id_class)    
        traffic_class_int = id_class - 1
		#rospy.logwarn("score-->",str(score[0]))
		#rospy.logwarn("traffic_class-->",str(traffic_class_int))
		#TODO implement light color prediction
	rospy.loginfo('Light state before return={}'.format(traffic_class_int))

	if (traffic_class_int == TrafficLight.RED):
		return TrafficLight.RED
	elif (traffic_class_int == TrafficLight.YELLOW):
		return TrafficLight.YELLOW
	elif (traffic_class_int == TrafficLight.GREEN):
		return TrafficLight.GREEN
	else:
		return TrafficLight.UNKNOWN
	   
