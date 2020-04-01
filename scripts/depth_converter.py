#!/usr/bin/env python

import rospy
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
from riptide_msgs.msg import Depth
import tf2_ros
from tf.transformations import quaternion_multiply, unit_vector, vector_norm, quaternion_conjugate, quaternion_matrix
from tf2_geometry_msgs import from_msg_msg
import math
import numpy as np

class depthConverter():
    def __init__(self):
        self.sub = rospy.Subscriber("depth/raw", Depth, self.depthCb)
        self.pub = rospy.Publisher("depth/pose", PoseWithCovarianceStamped, queue_size=10)
        self.namespace = rospy.get_namespace()[1:]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.b2pVector= None

    def depthCb(self, msg):
        try:
            # Rotation from base frame to odom
            b2oOrientation = self.tfBuffer.lookup_transform('odom', self.namespace+'base_link', rospy.Time()).transform.rotation
            b2oMatrix = quaternion_matrix([b2oOrientation.x, b2oOrientation.y, b2oOrientation.z, b2oOrientation.w])[:3,:3]

            if self.b2pVector is None:
                # Offset to pressure sensor
                pressureOffset = self.tfBuffer.lookup_transform(self.namespace+'pressure_link', self.namespace+'base_link', rospy.Time()).transform.translation
                self.b2pVector = [pressureOffset.x, pressureOffset.y, pressureOffset.z]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.loginfo(ex)
            return

        # Rotate pressure sensor offset into odom frame and get additional depth from offset
        # TODO: Calculate uncertainty of this measure
        addedDepth = np.dot(b2oMatrix, self.b2pVector)[2]

        # Publish z offset from odom to base_link (depth)
        outMsg = PoseWithCovarianceStamped()
        outMsg.header = msg.header
        outMsg.header.frame_id = "odom"
        outMsg.pose.pose.position.z = msg.depth + addedDepth
        outMsg.pose.covariance[14] = msg.variance
        self.pub.publish(outMsg)

if __name__ == '__main__':
    rospy.init_node('depth_converter')
    depthConverter()
    rospy.spin()
    