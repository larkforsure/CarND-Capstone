import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import os

class TLClassifier(object):
    def __init__(self, model_path):
        #TODO load classifier
        self.model_path = model_path

        # get the traffic light classifier
        config = tf.ConfigProto(log_device_placement=True)
        config.gpu_options.per_process_gpu_memory_fraction = 0.2  # don't hog all the VRAM!
        config.operation_timeout_in_ms = 50000 # terminate anything that don't return in 50 seconds
        self.tf_session = tf.Session(config=config)
        saver = tf.train.import_meta_graph(self.model_path + '/checkpoints/generator.ckpt.meta')
        saver.restore(self.tf_session, tf.train.latest_checkpoint(self.model_path + '/checkpoints/'))

        # get the tensors we need for doing the predictions by name
        tf_graph = tf.get_default_graph()
        self.tensor_input_real = tf_graph.get_tensor_by_name("input_real:0")
        self.tensor_drop_rate = tf_graph.get_tensor_by_name("drop_rate:0")
        self.tensor_predict = tf_graph.get_tensor_by_name("predict:0")



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        # scale image to 600x800
        image_scaled = image.reshape(-1, 600, 800 ,3)
        # scale features to (-1,1)
        min_val = image_scaled.min()
        image_scaled = (image_scaled - min_val) / (255 - min_val)
        image_scaled = image_scaled * 2 - 1.

        # all honor to John Chen
        predicted_state = [ TrafficLight.UNKNOWN ]
        predicted_state = self.tf_session.run(self.tensor_predict, feed_dict = {
                    self.tensor_input_real: image_scaled, 
                    self.tensor_drop_rate: 0. })

        #rospy.loginfo("prediction: %s", predicted_state)        
        return predicted_state[0]

