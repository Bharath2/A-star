#!/usr/bin/env python3

import rospy
import tf

from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class BotController(object):
    """
    A controller class to drive a turtlebot in Gazebo.
    """

    def __init__(self):
        self._velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self._robot_name = 'burger'
        self._velocity_msg = Twist()
        # set up a tf listener to retrieve transform between the robot and the world
        self._tf_listener = tf.TransformListener()

    def wait(self):
        self._tf_listener.waitForTransform('/odom', '/base_footprint',
                                            rospy.Time(), rospy.Duration(500))


    def get_transform(self):
        """
        Get the current pose of the robot in the world frame.
        """
        (trans, rot) = self._tf_listener.lookupTransform(
                         '/odom', '/base_footprint', rospy.Time(0))
        
        x, y, z = trans
        r, p, yaw = euler_from_quaternion(rot)

        return x, y, yaw

    def cmd_vel(self, linear, angular):
        """
        Publish linear and angular velocities to cmd_vel Topic.

        Args:
            linear (float): linear velocity
            angular (float): angular velocity
        """
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        self._velocity_publisher.publish(velocity)
