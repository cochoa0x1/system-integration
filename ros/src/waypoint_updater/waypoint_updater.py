#!/usr/bin/env python
from __future__ import print_function
import traceback

from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import numpy as np
import tf
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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber("/traffic_waypoint", Waypoint, self.traffic_cb)
        #rospy.Subscriber("/obstacle_waypoint", Waypoint, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = PoseStamped()
        self.waypoints = Lane()
        self.x = None
        self.y = None
        self.z = None
        self.theta = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            if self.x is None or self.waypoints.waypoints is None:
                rospy.logerr('waiting for init')
                rate.sleep()
                continue

            rospy.logerr('position: %f, %f, %f'%(self.x, self.y, self.theta))

            i = self.get_nearest_waypoint()
            new_wapoints = Lane()
            new_wapoints.waypoints = self.waypoints.waypoints[i:i+LOOKAHEAD_WPS]
            self.final_waypoints_pub.publish(new_wapoints)

            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        euler = tf.transformations.euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w])
        self.theta = euler[2]

        rospy.logerr('position: %f, %f'%(self.pose.pose.position.x, self.pose.pose.position.y))

        self.update_waypoints()
       
        #rospy.logerr(self.pose)
        
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def get_nearest_waypoint(self):
        '''returns the nearest waypoint index'''

        d = lambda b: np.sqrt((self.x-b.x)**2 + (self.y-b.y)**2  + (self.z-b.z)**2)
        
        min_dist = 1e12
        min_index = 0
        min_angle = 0

        for i, w in enumerate(self.waypoints.waypoints):

            pos = w.pose.pose.position
            dist = d(pos)

            heading = np.arctan2( self.y-pos.y, self.x - pos.x)

            theta = np.abs(heading - self.theta)
            
            if dist < min_dist and theta <= np.pi/4:
                min_dist = dist
                min_index = i
                min_angle = theta

        rospy.logerr('wp: %i, distance %f car_theta %f angle %f'%(min_index,min_dist,self.theta, min_angle))
        return min_index

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
        rospy.logerr(traceback.format_exc())
