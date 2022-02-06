#!/usr/bin/env python3

import rclpy
import rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
from riptide_hardware.msg import Depth
import tf2_ros
from tf.transformations import quaternion_multiply, unit_vector, vector_norm, quaternion_conjugate, quaternion_matrix
from tf2_geometry_msgs import from_msg_msg
import math
import numpy as np

class depthConverter(Node):
    super().__init__('riptide_localization2')
    def __init__(self):
        self.sub = self.create_subscription(Depth, "depth/raw", self.depthCb, qos_profile_system_default)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "depth/pose", qos_profile_system_default)
        self.namespace = self.get_namespace()[1:]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.b2pVector= None

    def depthCb(self, msg):
        try:
            # Rotation from base frame to odom
            b2oOrientation = self.tfBuffer.lookup_transform('odom', self.namespace+'base_link', rclpy.Time()).transform.rotation
            b2oMatrix = quaternion_matrix([b2oOrientation.x, b2oOrientation.y, b2oOrientation.z, b2oOrientation.w])[:3,:3]

            if self.b2pVector is None:
                # Offset to pressure sensor
                pressureOffset = self.tfBuffer.lookup_transform(self.namespace+'pressure_link', self.namespace+'base_link', rclpy.Time()).transform.translation
                self.b2pVector = [pressureOffset.x, pressureOffset.y, pressureOffset.z]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rclpy.loginfo(ex)
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

def main(args=None):
    rclpy.init(args=args)
    node = depthConverter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()   
    