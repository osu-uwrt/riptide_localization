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

        self.tommyDriftSub = self.create_subscription(PoseWithCovarianceStamped, "tommyDrift/pose", self.tommyDriftCb, qos_profile_sensor_data)
        self.tommySub = self.create_subscription(PoseWithCovarianceStamped, "{}mapping/tommy".format(self.get_namespace()), self.tommyCb, qos_profile_sensor_data)

        self.gmanDriftSub = self.create_subscription(PoseWithCovarianceStamped, "gmanDrift/pose", self.gmanDriftCb, qos_profile_sensor_data)
        self.gmanSub = self.create_subscription(PoseWithCovarianceStamped, "{}mapping/gman".format(self.get_namespace()), self.gmanCb, qos_profile_sensor_data)

        self.bootleggerDriftSub = self.create_subscription(PoseWithCovarianceStamped, "bootleggerDrift/pose", self.bootleggerDriftCb, qos_profile_sensor_data)
        self.bootleggerSub = self.create_subscription(PoseWithCovarianceStamped, "{}mapping/bootlegger".format(self.get_namespace()), self.bootleggerCb, qos_profile_sensor_data)

        self.badgeDriftSub = self.create_subscription(PoseWithCovarianceStamped, "badgeDrift/pose", self.badgeDriftCb, qos_profile_sensor_data)
        self.badgeSub = self.create_subscription(PoseWithCovarianceStamped, "{}mapping/badgd".format(self.get_namespace()), self.badgeCb, qos_profile_sensor_data)

        self.gateDriftSub = self.create_subscription(PoseWithCovarianceStamped, "gateDrift/pose", self.gateDriftCb, qos_profile_sensor_data)
        self.gateSub = self.create_subscription(PoseWithCovarianceStamped, "{}mapping/gate".format(self.get_namespace()), self.gateCb, qos_profile_sensor_data)

        self.odomSub = self.create_subscription(Odometry, "odometry/filtered", self.odomCb, qos_profile_sensor_data)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "odometry/drift_corrected", qos_profile_sensor_data)
        self.namespace = self.get_namespace()[1:]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.cutiePose = None
        self.tommyPose = None
        self.gmanPose = None
        self.bootleggerPose = None
        self.badgePose = None
        self.gatePose = None
        self.odomPoseMsg = None

    def cutieCb(self, msg):
        self.cutiePose = msg.pose.pose

    def tommyCb(self, msg):
        self.tommyPose = msg.pose.pose

    def gmanCb(self, msg):
        self.gmanPose = msg.pose.pose

    def bootleggerCb(self, msg):
        self.bootleggerPose = msg.pose.pose

    def badgeCb(self, msg):
        self.badgePose = msg.pose.pose

    def gateCb(self, msg):
        self.gatePose = msg.pose.pose

    def odomCb(self, msg):
        self.odomPoseMsg = msg

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

    def tommyDriftCb(self, msg):
        if self.odomPoseMsg is None:
            rclpy.loginfo("No initial map estimate for tommy")
            return

        odomPose = self.odomPoseMsg.pose.pose

        # Transform odom to tommy
        relOdomPose = self.tfBuffer.transform(odomPose, "{}mapping/tommy".format(self.get_namespace()))

        # TODO: This would probably be way easier if we just used tf
        driftedPose = msg.pose.pose
        correctedOdom = relOdomPose
        # translation from the drifted tommy to mapping's accepted tommy positiion
        driftTranslation = self.tommyPose.position - driftedPose.position
        driftRotation = tf3d.quaternions.qmult(xyzw_to_wxyz(self.tommyPose.rotation), tf3d.quaternions.qinverse(xyzw_to_wxyz(driftedPose.rotation))) 

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

    def gmanDriftCb(self, msg):
        if self.odomPoseMsg is None:
            rclpy.loginfo("No initial map estimate for gman")
            return

        odomPose = self.odomPoseMsg.pose.pose

        # Transform odom to gman
        relOdomPose = self.tfBuffer.transform(odomPose, "{}mapping/gman".format(self.get_namespace()))

        # TODO: This would probably be way easier if we just used tf
        driftedPose = msg.pose.pose
        correctedOdom = relOdomPose
        # translation from the drifted gman to mapping's accepted gman positiion
        driftTranslation = self.gmanPose.position - driftedPose.position
        driftRotation = tf3d.quaternions.qmult(xyzw_to_wxyz(self.gmanPose.rotation), tf3d.quaternions.qinverse(xyzw_to_wxyz(driftedPose.rotation))) 

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

    def bootleggerDriftCb(self, msg):
        if self.odomPoseMsg is None:
            rclpy.loginfo("No initial map estimate for bootlegger")
            return

        odomPose = self.odomPoseMsg.pose.pose

        # Transform odom to bootlegger
        relOdomPose = self.tfBuffer.transform(odomPose, "{}mapping/bootlegger".format(self.get_namespace()))

        # TODO: This would probably be way easier if we just used tf
        driftedPose = msg.pose.pose
        correctedOdom = relOdomPose
        # translation from the drifted bootlegger to mapping's accepted bootlegger positiion
        driftTranslation = self.bootleggerPose.position - driftedPose.position
        driftRotation = tf3d.quaternions.qmult(xyzw_to_wxyz(self.bootleggerPose.rotation), tf3d.quaternions.qinverse(xyzw_to_wxyz(driftedPose.rotation))) 

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

    def badgeDriftCb(self, msg):
        if self.odomPoseMsg is None:
            rclpy.loginfo("No initial map estimate for badge")
            return

        odomPose = self.odomPoseMsg.pose.pose

        # Transform odom to badge
        relOdomPose = self.tfBuffer.transform(odomPose, "{}mapping/badge".format(self.get_namespace()))

        # TODO: This would probably be way easier if we just used tf
        driftedPose = msg.pose.pose
        correctedOdom = relOdomPose
        # translation from the drifted badge to mapping's accepted badge positiion
        driftTranslation = self.badgePose.position - driftedPose.position
        driftRotation = tf3d.quaternions.qmult(xyzw_to_wxyz(self.badgePose.rotation), tf3d.quaternions.qinverse(xyzw_to_wxyz(driftedPose.rotation))) 

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

    def gateDriftCb(self, msg):
        if self.odomPoseMsg is None:
            rclpy.loginfo("No initial map estimate for gate")
            return

        odomPose = self.odomPoseMsg.pose.pose

        # Transform odom to gate
        relOdomPose = self.tfBuffer.transform(odomPose, "{}mapping/gate".format(self.get_namespace()))

        # TODO: This would probably be way easier if we just used tf
        driftedPose = msg.pose.pose
        correctedOdom = relOdomPose
        # translation from the drifted gate to mapping's accepted gate positiion
        driftTranslation = self.gatePose.position - driftedPose.position
        driftRotation = tf3d.quaternions.qmult(xyzw_to_wxyz(self.gatePose.rotation), tf3d.quaternions.qinverse(xyzw_to_wxyz(driftedPose.rotation))) 

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
        #ADD THIS POSE COVARIANCE TO A GLOBAL ARRAY
        #^
        #^


    

def xyzw_to_wxyz(q):
    x, y, z, w = q
    return (w,x,y,z)

def wxzy_to_xyzw(q):
    w, x, y, z = q
    return (x,y,z,w)

def calculateAvgPos(self, msg):
    #LOOP THROUGH ARRAY, GRAB COVARIANCES AND DO A WEIGHTED AVERAGE. ESTIMATE.PY DEF UPDATE_VALUE

def main(args=None):
    rclpy.init(args=args)
    node = dopeConverter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()   