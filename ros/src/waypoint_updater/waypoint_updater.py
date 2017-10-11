#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
from geometry_msgs.msg import TwistStamped
import math
import sys

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
ONE_MPH = 0.44704
MAX_SPEED = 40.0
SLOWDOWN_WPS = 50 # Number of waypoints before traffic light to start slowing down

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

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
        self.current_pose = None
        self.current_velocity = None
        self.last_wp_id = None
        self.lights = None

        rospy.loginfo("WaypointUpdater: initialize done")
        rospy.spin()

    def pose_cb(self, poseStamped):
        self.current_pose = poseStamped.pose
        rospy.loginfo("WaypointUpdater: Car position updated to %s", self.current_pose)
        self.update_waypoints()

    def waypoints_cb(self, lane):
        # publishes a list of all waypoints for the track and it only publishes once
        rospy.loginfo("WaypointUpdater: received base waypoints min %s, max %s", min([ wp.pose.pose.position.x for wp in lane.waypoints]), max([ wp.pose.pose.position.x for wp in lane.waypoints]));
        if self.waypoints is None:
            self.waypoints = lane.waypoints
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
        if ( self.waypoints is None or self.current_velocity is None
                or self.current_pose is None ):
            return

        car_vx = self.current_velocity.twist.linear.x
        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y

        # find the waypoint in front of
        next_id = -1
        begin = 0 
        end = len(self.waypoints)
        if self.last_wp_id is not None:
            begin = self.last_wp_id - 30
            end = min(end, self.last_wp_id + 60)

        for i in range(begin, end):
            wp_x = self.waypoints[i].pose.pose.position.x
            if wp_x > car_x : 
                next_id = i
                break
        
        if next_id == -1:  # back to the starting point
            next_id = 0

        self.last_wp_id = next_id
        rospy.loginfo("WaypointUpdater: next waypoint id %s, wp_x %s", next_id, self.waypoints[next_id].pose.pose.position.x)

        # fulfill LOOKAHEAD_WPS waypoints starting from next_id
        # tricky, firstly duplicate waypoints
        loop_waypoints = self.waypoints[:]
        loop_waypoints.extend(loop_waypoints)
        next_waypoints = loop_waypoints[next_id : next_id+LOOKAHEAD_WPS-1]

        # construct a Lane message
        lane = Lane()
        lane.waypoints = next_waypoints
        lane.header.stamp = rospy.get_rostime()
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
