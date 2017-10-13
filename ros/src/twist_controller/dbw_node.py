#!/usr/bin/env python
from __future__ import print_function
import traceback

import rospy
from std_msgs.msg import Bool
from styx_msgs.msg import Lane, Waypoint
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
import math

from twist_controller import Controller
from yaw_controller import YawController


from pid import PID

#common utility functions
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import utils

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        min_speed = .5

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.current_velocity = None
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb,queue_size=1)
        self.twist = None
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)

        self.pos = None
        
        self.controller = Controller()
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel,max_steer_angle)

        self.pid = PID(2.0, 0.4, 0.1,-1,1)
        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            throttle, brake, steering = self.controller.control()


            if self.twist is not None and self.current_velocity is not None:
                #rospy.logerr('target velocity: %f'%(self.twist.linear.x))
                #rospy.logerr('target vel %f'%self.twist.linear.x)
                #if self.twist.linear.x > self.current_velocity.linear.x:
                #    throttle = 1.0
                #else:
                #    throttle = 0.0
                #   #brake = 1000

                error = self.twist.linear.x - self.current_velocity.linear.x
                throttle = self.pid.step(error, 1.0/50)

                if throttle < 0:
                    brake = -1000*throttle - 100
                    throttle =0.0

                steering = self.yaw_controller.get_steering(self.twist.linear.x, self.twist.angular.z, self.current_velocity.linear.x)

                #rospy.logerr('%f,%f,%f'%(throttle, brake, steering))

            self.publish(throttle, brake, steering)
            rate.sleep()

    def pose_cb(self, msg):
        self.pos = utils.SimplePose(msg.pose)

    def current_velocity_cb(self,msg):
        self.current_velocity = msg.twist

        #self.current_linear_velocity = msg.twist.linear.x
        #self.current_angular_velocity = msg.twist.angular.z

    def twist_cb(self,msg):
        self.twist = msg.twist
        #self.linear_velocity = self.twist_cmd.twist.linear.x
        #self.angular_velocity = self.twist_cmd.twist.angular.z

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    try:
        DBWNode()
    except Exception as e:
        rospy.logerr("something failed in dbw_node")
        rospy.logerr(traceback.format_exc())
