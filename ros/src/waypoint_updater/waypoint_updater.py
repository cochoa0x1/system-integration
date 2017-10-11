#!/usr/bin/env python

from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import numpy as np

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
TARGET_SPEED = 20*.447


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
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

        rospy.logerr('position: %f, %f'%(self.pose.pose.position.x, self.pose.pose.position.y))

        self.update_waypoints()
       
        #rospy.logerr(self.pose)
        
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        #print(waypoints)

        #get the nearest waypoint
        # d = lambda a, b: np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        
        # pos = self.pose.pose.position
        # distances =[ d(pos,w.pose.pose.position) for w in waypoints.waypoints]

        # i = np.argmin(distances)
        # x =waypoints.waypoints[i].pose.pose.position.x
        # y =waypoints.waypoints[i].pose.pose.position.y
        # #y =w.twist.twist.linear.y
        
        # #publish the waypoints from i to i+LOOKAHEAD_WPS

    def update_waypoints(self):

        i = self.get_nearest_waypoint()
        new_wapoints = Lane()
        new_wapoints.waypoints = self.waypoints.waypoints[i:i+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(new_wapoints)


    def get_nearest_waypoint(self):
        '''returns the nearest waypoint index'''

        #TODO, should probably return the nearest waypoint faces in the car's direction

        min_dist = 1e12
        min_index = 0

        pos = self.pose.pose.position

        d = lambda a, b: np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        for i, w in enumerate(self.waypoints.waypoints):

            dist = d(pos,w.pose.pose.position)
            
            if dist < min_dist:
                min_dist = dist
                min_index = i

        return min_index


    def get_next_waypoints(self):
        '''returns the next waypoint the car should drive to,
            this is defined as the closest waypoint that is in the direction the car is facing

            how can we tell if the point is in front of the car?
            consturct two vectors: car_heading that points in the direction of the car
            and wd which is the vector with origin at carx,y, calculate the angle between them
        '''
        pos = self.pose.pose.position

        car_theta = self.pose.pose.orientation.z
        
        car_heading = np.array([np.cos(car_theta),np.sin(car_theta)])

        def calc_angle(w):
            b = np.array([w.pose.pose.position.x - pos.x, w.pose.pose.position.y - pos.y])
            b /= np.linalg.norm(b)

            return np.arccos(np.dot(car_heading,b))

        #get the nearest waypoint
        d = lambda a, b: np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        
        #filter down the distances to relvant angles
        min_dist = 1e12
        min_index = 0
        min_angle = 0
        for i, w in enumerate(self.waypoints.waypoints):

            dist = d(pos,w.pose.pose.position)
            theta = calc_angle(w)


            if np.abs(theta) <= np.pi/2 and dist < min_dist:
                min_dist = dist
                min_index = i
                min_angle = theta

        rospy.loginfo('clossest waypoint: %i, distance: %f min angle %f'%( min_index, min_dist, min_angle ))
        return self.waypoints.waypoints#[min_index:min_index+2]






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
