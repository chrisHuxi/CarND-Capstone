from styx_msgs.msg import TrafficLight
import rospy
import time
import tensorflow as tf
import numpy as np


class TLClassifier(object):
    def __init__(self,is_site):
        #TODO load classifier
        if is_site == False:
            self.Threshold_score = 0.7
            ssd_model = "/home/huxi/project/CarND-Capstone/ros/src/tl_detector/light_classification/frozen_model/frozen_sim_inception/frozen_inference_graph.pb"
        else:
            self.Threshold_score = 0.2
            print("is site!")
            ssd_model = "/home/huxi/project/CarND-Capstone/ros/src/tl_detector/light_classification/frozen_model/frozen_real_inception/frozen_inference_graph.pb"
        self.sess = None
        self.detection_graph = tf.Graph()

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(ssd_model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph)

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        rospy.loginfo("loaded ssd detector!")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        rospy.logwarn("!!!!")

        # TODO implement light color prediction
        id_color = TrafficLight.UNKNOWN
        image_np_expanded = np.expand_dims(image, axis=0)
        with self.detection_graph.as_default():
            scores_array, classes_array = self.sess.run([self.detection_scores, self.detection_classes], feed_dict={self.image_tensor: image_np_expanded})

        #1 'Green'
        #2 'Red'
        #3 'Yellow'
        #4 'off'
        rospy.logwarn("scores_array:{0}".format(scores_array))
        rospy.logwarn("classes_array:{0}".format(classes_array))
        
        rospy.logwarn("TrafficLight.RED:{0}".format(TrafficLight.RED))
        rospy.logwarn("TrafficLight.GREEN:{0}".format(TrafficLight.GREEN))
        rospy.logwarn("TrafficLight.YELLOW:{0}".format( TrafficLight.YELLOW))

        scores = np.array([s for s in scores_array[0] if s>self.Threshold_score])
        if len(scores) >= 1:
            classes = classes_array[0,0:len(scores)].astype('int32')
            print(classes)
            if (classes==2).any():
                id_color = TrafficLight.RED

            else:
                counts = np.bincount(classes)
                most_class = np.argmax(counts)
                if most_class == 1:
                    id_color = TrafficLight.GREEN
                elif most_class == 3:
                    id_color = TrafficLight.YELLOW

        return id_color

