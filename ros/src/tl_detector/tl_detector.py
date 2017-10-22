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

STATE_COUNT_THRESHOLD = 1
DETECT_DIST = 3600 # Dist**2 before next light to start image detection
LIGHTS_TABLE = ['RED', 'YELLOW', 'GREEN', 'N/A', 'UNKNOWN'] # refer to styx_msgs/msg/TrafficLight.msg

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

        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.lights_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        # lights nearest waypoint IDs
        self.stop_lights_wp_ids = []
        
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        model_path = rospy.get_param('~model_path')
        rospy.loginfo("TLDetector: Model path %s", model_path)
        #sys.stdout.flush()

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(model_path)
        self.listener = tf.TransformListener()

        # Let the car stop in the very beginning
        self.state_count = STATE_COUNT_THRESHOLD
        self.state = TrafficLight.RED 
        self.last_state = TrafficLight.UNKNOWN
        self.last_light_wp_id = -1

        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb) 
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        rospy.loginfo("TL Detection : Initialization done");
        rospy.loginfo("TL Detection : Good to go ~~~");
        sys.stdout.flush()

        rate = rospy.Rate( 4) # The camera rate in yaml_to_camera_info_publisher is 10   
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


    def pose_cb(self, poseStamped):
        self.current_pose = poseStamped.pose


    # publishes a list of all waypoints for the track and it only publishes once
    def waypoints_cb(self, lane):
        rospy.loginfo("TLDetector: Received base waypoints len %s, min_x %s, max_x %s", len(lane.waypoints), min([ wp.pose.pose.position.x for wp in lane.waypoints]), max([ wp.pose.pose.position.x for wp in lane.waypoints]));
        if self.waypoints is None:
            self.waypoints = lane.waypoints
            # FIXME there's a large gap between loop begin & end
            # Tricky, duplicate waypoints to deal with wrap problem
            self.loop_waypoints = self.waypoints[:]
            self.loop_waypoints.extend(self.loop_waypoints[:])

    def lights_cb(self, trafficLightArray):
        self.lights = trafficLightArray.lights


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg


    def loop(self):
        if ( self.current_pose is None or self.loop_waypoints is None 
                or self.light_classifier is None or self.camera_image is None ): 
            return

        light_wp_id, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        # Hack Yellow as Red
        #if state == TrafficLight.YELLOW:
        #    state = TrafficLight.RED 
        
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
        for i in range(len(self.waypoints)):
            wp_x = self.waypoints[i].pose.pose.position.x
            wp_y = self.waypoints[i].pose.pose.position.y
            dist = (pose.position.x - wp_x)**2 + (pose.position.y - wp_y)**2
            if dist < min_dist :
                min_dist = dist
                next_id = i
        return next_id



    def get_light_state(self):
        """Determines the current color of the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            # self.prev_light_loc = None
            return TrafficLight.RED
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        predicted_state = self.light_classifier.get_classification(cv_image)
        return predicted_state



    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Due to the tf inference lag, have to bring inference computation up. Or else the car_x, car_y will be actaully obsolete values, i.e. fall behind 
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

        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y

        # Find the closest visible traffic light (if one exists)
        min_dist = sys.maxsize
        next_id = None
        for i in range(len(stop_lights_positions)):
            # The last light position is totally worng, must map to wp_id, then caculate distance
            #wp_x = stop_lights_positions[i][0]
            #wp_y = stop_lights_positions[i][1]
            wp_x = self.waypoints[self.stop_lights_wp_ids[i]].pose.pose.position.x
            wp_y = self.waypoints[self.stop_lights_wp_ids[i]].pose.pose.position.y
            dist = (car_x - wp_x)**2 + (car_y - wp_y)**2
            if dist < min_dist :
                min_dist = dist
                next_id = i
       
        # Find the nearest dist bwteen two continuous light , car may be between them or behind them
        next_next_id = (next_id + 1) % len(self.lights)
        wp_x = self.waypoints[self.stop_lights_wp_ids[next_next_id]].pose.pose.position.x
        wp_y = self.waypoints[self.stop_lights_wp_ids[next_next_id]].pose.pose.position.y
        next_min_dist = (car_x - wp_x)**2 + (car_y - wp_y)**2
        min_dist = min(min_dist, next_min_dist)

        # Here is to minimize the inference caculation
        state = None
        if min_dist < DETECT_DIST: # We are still in moving during inference, so x2
            state = self.get_light_state()
        #if next_id == 6 or next_id == 7:
        #    rospy.loginfo("min_dist %s, car_x %s, car_y %s, wp_x %s, wp_y %s, next_id %s", min_dist, car_x, car_y, wp_x, wp_y, next_id)
        last_min_dist = min_dist

        # After inference lag, do all the caculation again !!!
        # The actual values after tf inference lag
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
            end = min(len(self.loop_waypoints), self.last_wp_id + 8) # search extend to duplicated loop waypoints

        for i in range(begin, end):
            wp_x = self.loop_waypoints[i].pose.pose.position.x
            wp_y = self.loop_waypoints[i].pose.pose.position.y
            dist = (car_x - wp_x)**2 + (car_y - wp_y)**2
            if dist < min_dist :
                min_dist = dist
                next_id = i

        #rospy.loginfo("TLDetector: next loop waypoint id %s, wp_x %s, wp_y %s", next_id, self.loop_waypoints[next_id].pose.pose.position.x, self.loop_waypoints[next_id].pose.pose.position.y)
        # Chop back
        self.last_wp_id = next_id if next_id < wp_len else next_id - wp_len

 
        # Find the closest visible traffic light (if one exists)
        min_dist = sys.maxsize
        next_id = None
        for i in range(len(stop_lights_positions)):
            #wp_x = stop_lights_positions[i][0]
            #wp_y = stop_lights_positions[i][1]
            wp_x = self.waypoints[self.stop_lights_wp_ids[i]].pose.pose.position.x
            wp_y = self.waypoints[self.stop_lights_wp_ids[i]].pose.pose.position.y
            dist = (car_x - wp_x)**2 + (car_y - wp_y)**2
            if dist < min_dist :
                # FIXME shall be road distance
                min_dist = dist
                next_id = i

        # Expect a light in front of the car
        # corner case: car is wrapping the starting point
        wp_len = len(self.waypoints)
        # case 1
        #is_car_wrap_1 = self.stop_lights_wp_ids[next_id] > 0.7 * wp_len \
        #                    and self.last_wp_id < 0.15 * wp_len
        is_car_wrap_1 = (next_id == len(stop_lights_positions)-1) and self.last_wp_id < self.stop_lights_wp_ids[0]
        # caes 2
        #is_car_wrap_2 = self.stop_lights_wp_ids[next_id] < 0.15 * wp_len \
        #                    and self.last_wp_id > 0.7 * wp_len
        is_car_wrap_2 = (next_id == 0) and self.last_wp_id >= self.stop_lights_wp_ids[-1]
        is_car_wrap = is_car_wrap_1 or is_car_wrap_2
        if is_car_wrap:
            rospy.loginfo("TLDetector: Car is wrapping the starting point, logic reversed")

        cmp_result = self.stop_lights_wp_ids[next_id] <= self.last_wp_id
        # In these two cases, the decision logic shall be revsered
        if is_car_wrap:
            cmp_result = not cmp_result

        if cmp_result:
            next_id = (next_id + 1) % len(self.lights)
            #wp_x = stop_lights_positions[next_id][0]
            #wp_y = stop_lights_positions[next_id][1]
            wp_x = self.waypoints[self.stop_lights_wp_ids[next_id]].pose.pose.position.x
            wp_y = self.waypoints[self.stop_lights_wp_ids[next_id]].pose.pose.position.y
            # FIXME shall be road distance
            min_dist = (car_x - wp_x)**2 + (car_y - wp_y)**2

        #rospy.loginfo("TLDetector: next light id %s, light_wp id %s, car_wp_id %s, cmp_result %s", next_id, self.stop_lights_wp_ids[next_id], self.last_wp_id, cmp_result)

        light = self.lights[next_id] # shall we rely on the existence of this topic?
        #if next_id == 6 or next_id == 7:
        #    rospy.loginfo("next_id %s, state %s, min_dist %s, cmp_result %s last_min_dist %s", next_id, state, min_dist, cmp_result, last_min_dist)
        if state is not None and min_dist < DETECT_DIST and not cmp_result:  # publish -1 when still far away from the next light
            
            rospy.loginfo("TLDetector:: predicted %s, ground truth %s, next light id %s", LIGHTS_TABLE[state], LIGHTS_TABLE[light.state], next_id)
            sys.stdout.flush()
            
            return self.stop_lights_wp_ids[next_id], state
        else:
            #??? self.waypoints = None
            return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
