#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
from geometry_msgs.msg import TwistStamped
import math, sys

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
SLOWDOWN_WPS = 50 # Number of waypoints before traffic light to start slowing down
SLOWDOWN_DIST = 1000 # Dist before traffic light to start slowing down

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.max_speed = 0.447*rospy.get_param('/waypoint_loader/velocity', 40.)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # temporary 
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.lights_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb) # this is base_waypoints plus traffic lights
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # the full track way points
        self.waypoints = None
        self.loop_waypoints = None
        self.current_pose = None
        self.current_velocity = None
        self.last_wp_id = None
        
        # temporary lights nearest waypoints IDs
        self.stop_lights_wp_ids = []
        self.lights = None
        
        rospy.spin()

    def pose_cb(self, poseStamped):
        self.current_pose = poseStamped.pose
        #rospy.loginfo("WaypointUpdater: Car position updated to %s", self.current_pose)
        self.update_waypoints()

    # publishes a list of all waypoints for the track and it only publishes once
    def waypoints_cb(self, lane):
        rospy.loginfo("WaypointUpdater: Received base waypoints size %s, min_x %s, max_x %s", len(lane.waypoints), min([ wp.pose.pose.position.x for wp in lane.waypoints]), max([ wp.pose.pose.position.x for wp in lane.waypoints]));
        if self.waypoints is None:
            self.waypoints = lane.waypoints
            # Tricky, duplicate waypoints to deal with wrap problem
            self.loop_waypoints = self.waypoints[:]
            self.loop_waypoints.extend(self.loop_waypoints)
            self.update_waypoints()

    def current_velocity_cb(self, twistStamped):
        self.current_velocity = twistStamped

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def lights_cb(self, trafficLightArray):
        self.lights = trafficLightArray.lights

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
    
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def update_waypoints(self):
        if ( self.loop_waypoints is None or self.current_velocity is None
                or self.current_pose is None or self.lights is None ):
            return

        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y

        # Find the nearest waypoint
        min_dist = sys.maxsize
        next_id = None
        wp_len = len(self.waypoints)
        begin = 0 
        end = wp_len # the first search shall be inside orignal waypoints
        if self.last_wp_id is not None:
            begin = max(0, self.last_wp_id - 1)
            end = min(end * 2, self.last_wp_id + 5) # search extend to duplicated loop waypoints

        for i in range(begin, end):
            wp_x = self.loop_waypoints[i].pose.pose.position.x
            wp_y = self.loop_waypoints[i].pose.pose.position.y
            dist = (car_x - wp_x)**2 + (car_y - wp_y)**2
            if dist < min_dist : 
                min_dist = dist
                next_id = i

        #rospy.loginfo("WaypointUpdater: next loop waypoint id %s, wp_x %s, wp_y %s", next_id, self.loop_waypoints[next_id].pose.pose.position.x, self.loop_waypoints[next_id].pose.pose.position.y)

        # Fulfill LOOKAHEAD_WPS waypoints starting from next_id
        next_waypoints = self.loop_waypoints[next_id : next_id+LOOKAHEAD_WPS-1]
        # chop back
        self.last_wp_id = next_id if next_id < wp_len else next_id - wp_len
      

        # One-time processing of lights_positions to lights nearest waypoint IDs
        if len(self.stop_lights_wp_ids) == 0:
            for light in self.lights:
                wp_id = self.get_closest_waypoint(light.pose.pose)
                self.stop_lights_wp_ids.append(wp_id)
            rospy.loginfo("TLDetector: Generated stop lights waypoint IDs %s", self.stop_lights_wp_ids)
            sys.stdout.flush()

        # Find the nearest light
        min_dist = sys.maxsize
        next_id = None
        for i in range(len(self.lights)-1):
            wp_x = self.lights[i].pose.pose.position.x
            wp_y = self.lights[i].pose.pose.position.y
            dist = (car_x - wp_x)**2 + (car_y - wp_y)**2
            if dist < min_dist : 
                min_dist = dist
                next_id = i

        # Expect a light in front of the car
        if(self.stop_lights_wp_ids[next_id] < self.last_wp_id):
            next_id = (next_id + 1) % len(self.lights)
            wp_x = self.lights[next_id].pose.pose.position.x
            wp_y = self.lights[next_id].pose.pose.position.y
            min_dist = (car_x - wp_x)**2 + (car_y - wp_y)**2

        # UNKNOWN=4, GREEN=2, YELLO=1, RED=0
        #rospy.loginfo("WaypointUpdater: next light id %s, light_wp id %s, light_x %s, light_y %s, state %s, min_dist %s", next_id, self.stop_lights_wp_ids[next_id], self.lights[next_id].pose.pose.position.x, self.lights[next_id].pose.pose.position.y, self.lights[next_id].state, min_dist)

        is_red_light = ( self.lights[next_id].state == TrafficLight.RED or self.lights[next_id].state == TrafficLight.YELLOW ) and ( min_dist < SLOWDOWN_DIST )
        rospy.loginfo("WaypointUpdater: RED?=%s", is_red_light)
       
        car_vx = self.current_velocity.twist.linear.x
        # Adjust next_waypoints velocity
        for i in range(len(next_waypoints)-1):
            if not is_red_light:
                self.set_waypoint_velocity(next_waypoints, i, self.max_speed) 
            else:
                target_v = 0; #car_vx - (i+1)*(car_vx/len(next_waypoints)) 
                self.set_waypoint_velocity(next_waypoints, i, target_v)

        # construct a Lane message
        lane = Lane()
        lane.waypoints = next_waypoints
        lane.header.stamp = rospy.get_rostime()
        self.final_waypoints_pub.publish(lane)


    def get_closest_waypoint(self, pose):
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
