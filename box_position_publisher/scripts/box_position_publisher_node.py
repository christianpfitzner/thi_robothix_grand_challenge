#!/usr/bin/env python

# This script is responsible for publishing the position of the box to the ros topic /box_position
#
# The position is calculated from 3 points that are input by the user.
# The points are used to calculate the origin of the box as well as the orientation of the box.

import roslib
import rospy
import tf
from geometry_msgs.msg import PoseStamped
import numpy as np

def pose_tf_broadcast(msg):
    
    # tf broadcaster
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                     tf.transformations.quaternion_from_euler(0, 0, msg.pose.orientation.z),
                     rospy.Time.now(),
                     "box_base_link",
                     "world")


# Input from 
def point_input():

    x = float(input("  x: "))
    y = float(input("  y: "))
    z = float(input("  z: "))

    return np.array([x, y, z])


def calc_origin(p1, p2, p3):
    # Calculate the vector from p1 to p2
    v1 = p2 - p1

    # Calculate the vector from p1 to p3
    v2 = p3 - p1

    # Calculate the cross product of v1 and v2
    v3 = np.cross(v1, v2)

    # unit vectors
    e_y = - v1 / np.linalg.norm(v1)
    e_z = v3 / np.linalg.norm(v3)
    e_x = np.cross(e_y, e_z)

    lambda1 = ((p1[1] - p3[1])*e_x[0] + (p3[0] - p1[0])*e_x[1])/(e_x[1]*e_y[0] - e_x[0]*e_y[1])

    origin = p1 + lambda1*e_y

    return origin

def calc_z_angle(p1, p2, p3):

    v_x = p2 - p1

    e_x = v_x / np.linalg.norm(v_x)

    e_x_world = np.array([1, 0, 0])

    return np.arccos(np.dot(e_x, e_x_world))

if __name__ == "__main__":

    # Initialize the node
    rospy.init_node('box_position_publisher_node')

    # Get the 3 points from the user
    print("Point 1:")
    p1 = point_input()

    print("Point 2:")
    p2 = point_input()

    print("Point 3:")
    p3 = point_input()

    print("Origin: ", calc_origin(p1, p2, p3))

    # Create a message
    msg = PoseStamped()

    # Set the position
    msg.pose.position.x = calc_origin(p1, p2, p3)[0]
    msg.pose.position.y = calc_origin(p1, p2, p3)[1]
    msg.pose.position.z = calc_origin(p1, p2, p3)[2]

    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = calc_z_angle(p1, p2, p3)

    msg.header.frame_id = "world"

    # Broadcast tf world -> box_base_link
    while not rospy.is_shutdown():
        pose_tf_broadcast(msg)
        rospy.sleep(0.1)