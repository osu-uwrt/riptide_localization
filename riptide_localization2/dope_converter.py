#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from tf2_ros import *
import transforms3d as tf3d
import numpy as np

class dopeConverter(Node):
    def __init__(self):
        super().__init__('riptide_localization2')
        
        # TODO: check topic names
        # TODO: Read all dope objects
        self.cutieDriftSub = self.create_subscription(PoseWithCovarianceStamped, "cutieDrift/pose", self.cutieDriftCb, qos_profile_sensor_data)
        self.cutieSub = self.create_subscription(PoseWithCovarianceStamped, "{}mapping/cutie".format(self.get_namespace()), self.cutieCb, qos_profile_sensor_data)
        self.odomSub = self.create_subscription(Odometry, "odometry/filtered", self.odomCb, qos_profile_sensor_data)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "odometry/drift_corrected", qos_profile_sensor_data)
        self.namespace = self.get_namespace()[1:]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.cutiePose = None
        self.odomPoseMsg = None

    def cutieCb(self, msg):
        self.cutiePose = msg.pose.pose

    def odomCb(self, msg):
        self.odomPoseMsg = msg.pose.pose

    def cutieDriftCb(self, msg):
        if self.odomPoseMsg is None:
            rclpy.loginfo("No initial map estimate for cutie")
            return

        odomPose = self.odomPoseMsg.pose.pose

        # Transform odom to cutie
        relOdomPose = self.tfBuffer.transform(odomPose, "{}mapping/cutie".format(self.get_namespace()))

        # TODO: This would probably be way easier if we just used tf
        driftedPose = msg.pose.pose
        correctedOdom = relOdomPose
        # translation from the drifted cutie to mapping's accepted cutie positiion
        driftTranslation = self.cutiePose.position - driftedPose.position
        driftRotation = tf3d.quaternions.qmult(xyzw_to_wxyz(self.cutiePose.rotation), tf3d.quaternions.qinverse(xyzw_to_wxyz(driftedPose.rotation))) 

        driftCorrectionMatrix = tf3d.affines.compose(driftTranslation, tf3d.quaternions.quat2mat(driftRotation))
        # Transform the odometry pose relative to the corrected odom
        correctedOdom.position = np.dot(driftCorrectionMatrix, relOdomPose.position)
        correctedOdom.rotation = wxzy_to_xyzw(tf3d.quaternions.qmult(relOdomPose.rotation, driftRotation))
        
        # Tranform corrected odom back to world
        correctedWorldOdom = self.tfBuffer.transform(odomPose, "world")

        # Create corrected odom message
        pubMsg = PoseWithCovarianceStamped()
        pubMsg.header.frame_id = "world"
        pubMsg.header.stamp = Time().to_msg()
        pubMsg.pose.pose = correctedWorldOdom
        # Currently covariance is the same as the error pose
        # Maybe it should be different?
        pubMsg.pose.covariance = self.odomPoseMsg.pose.covariance
        self.pub.publish(pubMsg)

def xyzw_to_wxyz(q):
    x, y, z, w = q
    return (w,x,y,z)

def wxzy_to_xyzw(q):
    w, x, y, z = q
    return (x,y,z,w)

def main(args=None):
    rclpy.init(args=args)
    node = dopeConverter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()   