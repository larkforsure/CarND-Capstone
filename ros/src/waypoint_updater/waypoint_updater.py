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
SLOWDOWN_DIST = 30

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.max_speed = rospy.get_param('/waypoint_loader/velocity', 40.)
        #rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.lights_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.upcoming_red_light_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # the full track way points
        self.waypoints = None
        self.loop_waypoints = None
        self.current_pose = None
        self.current_velocity = None
        self.last_wp_id = None
        self.upcoming_red_light_wp_id = None
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
   
        rate = rospy.Rate(12) 
        while not rospy.is_shutdown():
            self.update_waypoints()
            rate.sleep()


    def pose_cb(self, poseStamped):
        self.current_pose = poseStamped.pose
        #rospy.loginfo("WaypointUpdater: Car position updated to %s", self.current_pose)
        #self.update_waypoints()


    # publishes a list of all waypoints for the track and it only publishes once
    def waypoints_cb(self, lane):
        rospy.loginfo("WaypointUpdater: Received base waypoints len %s, min_x %s, max_x %s", len(lane.waypoints), min([ wp.pose.pose.position.x for wp in lane.waypoints]), max([ wp.pose.pose.position.x for wp in lane.waypoints]));
        if self.waypoints is None:
            self.waypoints = lane.waypoints
            # FIXME there's a large gap between loop begin & end
            # Tricky, duplicate waypoints to deal with wrap problem
            self.loop_waypoints = self.waypoints[:]
            self.loop_waypoints.extend(self.loop_waypoints[:])


    def current_velocity_cb(self, twistStamped):
        self.current_velocity = twistStamped

    def upcoming_red_light_cb(self, light_wp_id):
        self.upcoming_red_light_wp_id = light_wp_id.data

    def lights_cb(self, trafficLightArray):
        #self.lights = trafficLightArray.lights
        pass

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
                or self.current_pose is None or self.upcoming_red_light_wp_id is None ):
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
            begin = max(0, self.last_wp_id - 2)
            end = min(len(self.loop_waypoints), self.last_wp_id + 8) # search extend to duplicated loop waypoints

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
        # Chop back
        self.last_wp_id = next_id if next_id < wp_len else next_id - wp_len
      
        #rospy.loginfo("WaypointUpdater: car_wp_id %s, red_light_wp_id %s", self.last_wp_id, self.upcoming_red_light_wp_id)
        #sys.stdout.flush()
        
        # Adjust next_waypoints velocity
        car_vx = self.current_velocity.twist.linear.x
        car_dist = self.distance(self.waypoints, self.last_wp_id, self.upcoming_red_light_wp_id)
        for i in range(len(next_waypoints)):
            target_v = None
            if self.upcoming_red_light_wp_id == -1 or car_dist < 1e-5: 
                target_v = self.max_speed
            else:
                last_wp_id_i = self.last_wp_id + i
                last_wp_id_i = last_wp_id_i if last_wp_id_i < wp_len else last_wp_id_i - wp_len
                # Distance along the road
                dist = self.distance(self.waypoints, last_wp_id_i, self.upcoming_red_light_wp_id)
                low_bound = 0.5
                up_bound = SLOWDOWN_DIST + low_bound
                #rospy.loginfo("dist %s", dist)
                #sys.stdout.flush()
                if car_dist < up_bound:  # What shall I do?
                    target_v = self.get_waypoint_velocity(next_waypoints[i])
                    if target_v > self.max_speed or target_v < 0:
                        rospy.loginfo("WARNING, i %s, target_v %s", i, target_v)
                elif dist < low_bound:
                    target_v = 0
                elif dist < up_bound: 
                    target_v = (self.max_speed / 4) * ((dist - low_bound) / (up_bound - low_bound))
                else:
                    target_v = self.max_speed

            self.set_waypoint_velocity(next_waypoints, i, target_v)

        # Construct a Lane message
        lane = Lane()
        lane.waypoints = next_waypoints
        lane.header.stamp = rospy.get_rostime()
        self.final_waypoints_pub.publish(lane)


    def get_closest_waypoint(self, pose):
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

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
