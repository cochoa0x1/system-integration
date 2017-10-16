#!/usr/bin/env python
from __future__ import print_function
import traceback

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import numpy as np
import tf

#common utility functions
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import utils


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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.waypoints = None
        self.pos = None

        self.light_i = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            if self.pos is None or self.waypoints is None:
                rospy.logerr('waiting for init')
                rate.sleep()
                continue


            
            i = utils.get_nearest_waypoint(self.pos,self.waypoints)
            if self.light_i is None:
                self.light_i = -1
            rospy.logerr('%s next wp: %i ,next light: %i'%(self.pos,i, self.light_i))
            
            new_wapoints = Lane()


            cutoff = i+LOOKAHEAD_WPS

            TARGET_VEL = 10*.4457

            new_wapoints.waypoints = self.waypoints[i:cutoff]

            #if we have a red light ahead of us, slow down
            if self.light_i - i < 75 and self.light_i >= i:
                rospy.logerr('RED LIGHT AHEAD! SLAMM THE BRAKES DUDE!')
                TARGET_VEL = 0


            #set the speed for these waypoints
            for k,w in enumerate(new_wapoints.waypoints):
                w.twist.twist.linear.x = TARGET_VEL

            self.final_waypoints_pub.publish(new_wapoints)

            rate.sleep()

    def pose_cb(self, msg):
        self.pos = utils.SimplePose(msg.pose)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

        self.base_wp_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.light_i = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
        rospy.logerr(traceback.format_exc())
