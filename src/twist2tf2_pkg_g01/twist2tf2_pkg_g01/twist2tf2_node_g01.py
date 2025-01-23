#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist

import math
import numpy as np
from numpy import cos, sin

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def rotz(ang):

    R = np.array([[cos(ang),-sin(ang),0],
                  [sin(ang),cos(ang),0],
                  [0,0,1]])

    return R

def map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class Twist2TF2G01(Node):

    def __init__(self):
        super().__init__('twist2TF2_g01')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # callback function on each message
        self.subscription = self.create_subscription(Twist,'/twist_msg',self.twist2tf2_callback,1)

        self.subscription  # prevent unused variable warning

        # initial condition euler integration method
        self.dt = 0.1
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0

        self.u = 0.0
        self.v = 0.0
        self.r = 0.0

        # initialization of the transformation
        self.t = TransformStamped()
        self.t.header.frame_id = 'world'
        self.t.child_frame_id = 'odom'

        # initialization of the transformation
        self.t2 = TransformStamped()
        self.t2.header.frame_id = 'odom'
        self.t2.child_frame_id = 'base_link'

        # create a timer for the  car velocity publisher
        self.timer_broadcaster = self.create_timer(self.dt,self.broadcaster_callback)

    def broadcaster_callback(self):

        # vector form 
        xi = np.array([[self.u],[self.v],[self.r]])

        # rotating the position vector to correspond with the orientation
        eta_dot = rotz(self.theta)@xi

        # time stamp from ROS timer (odom2world)
        self.t.header.stamp = self.get_clock().now().to_msg()

        # time stamp from ROS timer (B2odom)
        self.t2.header.stamp = self.get_clock().now().to_msg()

        # Integrating angular velocity to angular position
        self.theta  = self.theta + eta_dot[2,0]*self.dt

        # Transformation angular position Euler Angles to Quaternion 
				# (odom2world) orientation
        self.t.transform.rotation.x = 0.0 
        self.t.transform.rotation.y = 0.0 
        self.t.transform.rotation.z = 0.0 
        self.t.transform.rotation.w = 1.0 
				# this is equivalent to the Identity Matrix in quaternions forms 

        # Transformation angular position Euler Angles to Quaternion
				# (B2odom) orientation
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        self.t2.transform.rotation.x = q[0]
        self.t2.transform.rotation.y = q[1]
        self.t2.transform.rotation.z = q[2]
        self.t2.transform.rotation.w = q[3]

        # Integrating linear velocity to linear position
        self.x = self.x +  eta_dot[0,0]*self.dt  
			   # this always is zero, but only for this example, z also is zero for this example.
        self.y = self.y +  eta_dot[1,0]*self.dt 

				# (odom2world) translation
        self.t.transform.translation.x = float(self.x)
        self.t.transform.translation.y = float(self.y)
        self.t.transform.translation.z = 0.0

				# (B2odom) translation
        self.t2.transform.translation.x = 0.0
        self.t2.transform.translation.y = 0.0
        self.t2.transform.translation.z = 0.0

        # Send the transformation (odom2world)
        self.tf_broadcaster.sendTransform(self.t)

        # Send the transformation (B2odom)
        self.tf_broadcaster.sendTransform(self.t2)

    def twist2tf2_callback(self, msg):

        # read the msg and extract the content
        self.u = map(msg.linear.x,0,255,-5,5)
        self.v = msg.linear.y 
        self.r = map(msg.angular.z,0,255,-5,5)

        print("new u: {:.4f}, r: {:.4f}".format(self.u,self.r))


def main():
    rclpy.init()
    node = Twist2TF2G01()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()