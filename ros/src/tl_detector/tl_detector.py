#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import numpy as np
import yaml,math,sys

STATE_COUNT_THRESHOLD = 3
SLOWDOWN_DIST = 3000 # Dist**2 before traffic light to start slowing down

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.current_pose = None
        self.waypoints = None
        self.loop_waypoints = None
        self.last_wp_id = None
        self.camera_image = None
        self.lights = []

        self.sim_testing = bool(rospy.get_param("~sim_testing", True))
        threshold = rospy.get_param('~threshold', 0.3)

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.lights_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=2*52428800) # extended

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        # lights nearest waypoint IDs
        self.stop_lights_wp_ids = []
        
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_light_wp_id = -1
        self.state_count = 0

        rospy.loginfo("TL Detection : Initialization done");

        rospy.spin()


    def pose_cb(self, poseStamped):
        self.current_pose = poseStamped.pose


    # publishes a list of all waypoints for the track and it only publishes once
    def waypoints_cb(self, lane):
        rospy.loginfo("TLDetector: Received base waypoints len %s, min_x %s, max_x %s", len(lane.waypoints), min([ wp.pose.pose.position.x for wp in lane.waypoints]), max([ wp.pose.pose.position.x for wp in lane.waypoints]));
        if self.waypoints is None:
            self.waypoints = lane.waypoints
            # Tricky, duplicate waypoints to deal with wrap problem
            self.loop_waypoints = self.waypoints[:]
            self.loop_waypoints.extend(self.loop_waypoints)


    def lights_cb(self, trafficLightArray):
        self.lights = trafficLightArray.lights


    def image_cb(self, msg):
        if ( self.current_pose is None or self.loop_waypoints is None ) :
            return
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp_id, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp_id = light_wp_id if state == TrafficLight.RED else -1
            self.last_light_wp_id = light_wp_id
            self.upcoming_red_light_pub.publish(Int32(light_wp_id))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_light_wp_id))
        self.state_count += 1


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

            """
        #TODO implement
        min_dist = sys.maxsize
        next_id = None
        for i in range(len(self.waypoints) - 1):
            wp_x = self.waypoints[i].pose.pose.position.x
            wp_y = self.waypoints[i].pose.pose.position.y
            dist = (pose.position.x - wp_x)**2 + (pose.position.y - wp_y)**2
            if dist < min_dist :
                min_dist = dist
                next_id = i
        return next_id

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # temporary
        return light.state
        #Get classification
        #return self.light_classifier.get_classification(cv_image)



    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y
        
        # Fast algorithm, find the nearest waypoint
        min_dist = sys.maxsize
        next_id = None
        wp_len = len(self.waypoints)
        begin = 0
        end = wp_len # the first search shall be inside orignal waypoints
        if self.last_wp_id is not None:
            begin = max(0, self.last_wp_id - 2)
            end = min(end * 2, self.last_wp_id + 8) # search extend to duplicated loop waypoints

        for i in range(begin, end):
            wp_x = self.loop_waypoints[i].pose.pose.position.x
            wp_y = self.loop_waypoints[i].pose.pose.position.y
            dist = (car_x - wp_x)**2 + (car_y - wp_y)**2
            if dist < min_dist :
                min_dist = dist
                next_id = i

        #rospy.loginfo("TLDetector: next loop waypoint id %s, wp_x %s, wp_y %s", next_id, self.loop_waypoints[next_id].pose.pose.position.x, self.loop_waypoints[next_id].pose.pose.position.y)
        # chop back
        self.last_wp_id = next_id if next_id < wp_len else next_id - wp_len

 
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_lights_positions = self.config['stop_line_positions']
        # One-time processing of lights_positions to lights nearest waypoint IDs
        if len(self.stop_lights_wp_ids) == 0:
            for p in stop_lights_positions:
                pose = Pose()
                pose.position.x = p[0]
                pose.position.y = p[1]
                wp_id = self.get_closest_waypoint(pose)
                self.stop_lights_wp_ids.append(wp_id)
            rospy.loginfo("TLDetector: Generated stop lights waypoint IDs %s", self.stop_lights_wp_ids)    
            sys.stdout.flush()


        # Find the closest visible traffic light (if one exists)
        min_dist = sys.maxsize
        next_id = None
        for i in range(len(stop_lights_positions)-1):
            wp_x = stop_lights_positions[i][0]
            wp_y = stop_lights_positions[i][1]
            dist = (car_x - wp_x)**2 + (car_y - wp_y)**2
            if dist < min_dist :
                min_dist = dist
                next_id = i

        # Expect a light in front of the car
        # corner case: car is wrapping the starting point
        wp_len = len(self.waypoints)
        # case 1
        is_car_wrap_1 = self.stop_lights_wp_ids[next_id] > 0.9 * wp_len \
                            and self.last_wp_id < 0.1 * wp_len
        # caes 2
        is_car_wrap_2 = self.stop_lights_wp_ids[next_id] < 0.1 * wp_len \
                            and self.last_wp_id > 0.9 * wp_len
        is_car_wrap = is_car_wrap_1 or is_car_wrap_2
        if is_car_wrap:
            rospy.loginfo("TLDetector: Car wrapping starting point, logic reversed")

        # in these two cases, the decision logic shall be revsered
        cmp_result = self.stop_lights_wp_ids[next_id] < self.last_wp_id
        if is_car_wrap:
            cmp_result = not cmp_result

        if cmp_result:
            next_id = (next_id + 1) % len(self.lights)
            wp_x = stop_lights_positions[next_id][0]
            wp_y = stop_lights_positions[next_id][1]
            min_dist = (car_x - wp_x)**2 + (car_y - wp_y)**2

        # UNKNOWN=4, GREEN=2, YELLO=1, RED=0
        #rospy.loginfo("TLDetector: next light id %s, light_wp id %s, light_x %s, light_y %s, min_dist %s, car_wp_id, %s, car_wrap %s", next_id, self.stop_lights_wp_ids[next_id], stop_lights_positions[next_id][0], stop_lights_positions[next_id][1], min_dist, self.last_wp_id, is_car_wrap)

        light = self.lights[next_id] # shall we rely on the existence of this topic?

        if light and min_dist < SLOWDOWN_DIST:  # publish -1 when still far away from the next light
            state = self.get_light_state(light)
            
            is_red_light = ( state == TrafficLight.RED ) and ( min_dist < SLOWDOWN_DIST )
            rospy.loginfo("TLDetector:: RED?=%s, next light id %s", is_red_light, next_id)
            return self.stop_lights_wp_ids[next_id], state
        else:
            #??? self.waypoints = None
            return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
