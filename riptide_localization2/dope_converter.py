#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from tf2_ros import *
import transforms3d as tf3d
import numpy as np


objects = {
    "cutie": { # Old game object used for testing
        "driftSub": None,
        "sub": None,
        "mappingPose": None,
        "correctedOdomPub": None,
    },
    "tommy": { # Tommygun game object.
        "driftSub": None,
        "sub": None,
        "mappingPose": None,
        "correctedOdomPub": None,
    },
    "gman": { # GMan game object.
        "driftSub": None,
        "sub": None,
        "mappingPose": None,
        "correctedOdomPub": None,
    },
    "bootlegger": { # Bootlegger game object.
        "driftSub": None,
        "sub": None,
        "mappingPose": None,
        "correctedOdomPub": None,
    },
    "badge": { # Badge game object.
        "driftSub": None,
        "sub": None,
        "mappingPose": None,
        "correctedOdomPub": None,
    },
    "gate": {
        "driftSub": None,
        "sub": None,
        "mappingPose": None,
        "correctedOdomPub": None,
    },
}

class dopeConverter(Node):
    def __init__(self):
        super().__init__('riptide_localization2')
        
        # initialize objects
        for k in objects.keys(): 
            objects[k]["driftSub"] = self.create_subscription(PoseWithCovarianceStamped, "{}Drift/pose".format(k), lambda msg: self.driftCb(self, msg, k), qos_profile_sensor_data)
            objects[k]["sub"] = self.create_subscription(PoseWithCovarianceStamped, "mapping/{}".format(k), lambda msg: self.mappingCb(self, msg, k), qos_profile_sensor_data)
            objects[k]["correctedOdomPub"] = self.create_publisher(PoseWithCovarianceStamped, "drift_corrected_pose/{}".format(k), qos_profile_sensor_data)

        self.odomSub = self.create_subscription(Odometry, "odometry/filtered", self.odomCb, qos_profile_sensor_data)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "odometry/drift_corrected", qos_profile_sensor_data)
        self.namespace = self.get_namespace()[1:]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.odomPoseMsg = None
        self.corrected_pose

    # Gets accepted mapping position
    def mappingCb(self, msg, k):
        objects[k]["mappingPose"] = msg.pose
    
    def odomCb(self, msg):
        self.odomPoseMsg = msg
    
    def driftCb(self, msg, k):
        '''
        driftCb corrects position of the robot based on mappings excepted position and recent dope 
        
        :param msg: - The drifted position of the object that mapping rejected
        :param k: - The drifted object (i.e. gate, cutie, bootlegger) 
        '''

        if self.odomPoseMsg is None:
            rclpy.loginfo("No initial estimate for robot pose. Skipping drift correction")
            return
        
        if (objects[k]["mappingPose"] is None):
            rclpy.loginfo("No initial map estimate for {}. Skipping drift correction".format(k))

        odomPose = self.odomPoseMsg.pose.pose

        # Get robot Position relative to the object
        relOdomPose = self.tfBuffer.transform(odomPose, "{}mapping/{}".format(self.get_namespace(), k))

        driftedPose = msg.pose.pose
        correctedOdom = relOdomPose
        # Find difference between drifted object detection to mapping's accepted position of the object
        driftTranslation = objects[k]["mappingPose"].position - driftedPose.position
        driftRotation = tf3d.quaternions.qnorm(tf3d.quaternions.qmult(self.xyzw_to_wxyz(objects[k]["mappingPose"].rotation), tf3d.quaternions.qinverse(self.xyzw_to_wxyz(driftedPose.rotation))))

        driftCorrectionMatrix = tf3d.affines.compose(driftTranslation, tf3d.quaternions.quat2mat(driftRotation))
        # Transform the odometry pose relative to the corrected odom
        correctedOdom.position = np.dot(driftCorrectionMatrix, relOdomPose.position)
        rot = tf3d.quaternions.qmult(relOdomPose.rotation, driftRotation)
        correctedOdom.rotation = self.wxyz_to_xyzw(tf3d.quaternions.qnorm(rot))
        
        # Tranform corrected odom back to world
        correctedWorldOdom = self.tfBuffer.transform(odomPose, "world")

        # Create corrected odom message
        correctedOdomMsg = msg
        correctedOdomMsg.header.stamp = Time().to_msg()
        correctedOdomMsg.pose.pose = correctedWorldOdom
        correctedOdomMsg.covariance = msg.pose.covariance # TODO(hunter): Transform old covariance matrix
        objects[k]["correctedOdomPub"].publish(correctedOdomMsg)

# Utility functions, since tf3d doesn't offer ways to convert between quaternion representations :(
    def xyzw_to_wxyz(self, q):
        x, y, z, w = q
        return (w,x,y,z)

    def wxyz_to_xyzw(self, q):
        w, x, y, z = q
        return (x,y,z,w)

def main(args=None):
    rclpy.init(args=args)
    node = dopeConverter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()   