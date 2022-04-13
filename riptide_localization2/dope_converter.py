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
        "correctedOdomPose": None,
    },
    "tommy": { # Tommygun game object.
        "driftSub": None,
        "sub": None,
        "mappingPose": None,
        "correctedOdomPose": None,
    },
    "gman": { # GMan game object.
        "driftSub": None,
        "sub": None,
        "mappingPose": None,
        "correctedOdomPose": None,
    },
    "bootlegger": { # Bootlegger game object.
        "driftSub": None,
        "sub": None,
        "mappingPose": None,
        "correctedOdomPose": None,
    },
    "badge": { # Badge game object.
        "driftSub": None,
        "sub": None,
        "mappingPose": None,
        "correctedOdomPose": None,
    },
    "gate": {
        "driftSub": None,
        "sub": None,
        "mappingPose": None,
        "correctedOdomPose": None,
    },
}

class dopeConverter(Node):
    def __init__(self):
        super().__init__('riptide_localization2')

        for k in objects.keys(): # iterate through keys
            objects[k]["driftSub"] = self.create_subscription(PoseWithCovarianceStamped, "{}Drift/pose".format(k), lambda msg: driftCb(self, msg, k), qos_profile_sensor_data)
            objects[k]["sub"] = self.create_subscription(PoseWithCovarianceStamped, "mapping/{}".format(k), lambda msg: mappingCb(self, msg, k), qos_profile_sensor_data)

        self.odomSub = self.create_subscription(Odometry, "odometry/filtered", self.odomCb, qos_profile_sensor_data)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "odometry/drift_corrected", qos_profile_sensor_data)
        self.namespace = self.get_namespace()[1:]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.odomPoseMsg = None

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

        # Transform odom to the object
        relOdomPose = self.tfBuffer.transform(odomPose, "{}mapping/{}".format(self.get_namespace(), k))

        driftedPose = msg.pose.pose
        correctedOdom = relOdomPose
        # translation from the drifted object to mapping's accepted position of the object
        driftTranslation = objects[k]["mappingPose"].position - driftedPose.position
        driftRotation = tf3d.quaternions.qmult(xyzw_to_wxyz(objects[k]["mappingPose"].rotation), tf3d.quaternions.qinverse(xyzw_to_wxyz(driftedPose.rotation))) 

        driftCorrectionMatrix = tf3d.affines.compose(driftTranslation, tf3d.quaternions.quat2mat(driftRotation))
        # Transform the odometry pose relative to the corrected odom
        correctedOdom.position = np.dot(driftCorrectionMatrix, relOdomPose.position)
        correctedOdom.rotation = wxzy_to_xyzw(tf3d.quaternions.qmult(relOdomPose.rotation, driftRotation))
        
        # Tranform corrected odom back to world
        correctedWorldOdom = self.tfBuffer.transform(odomPose, "world")

        # Create corrected odom message
        objects[k]["correctedOdomPose"] = PoseWithCovariance()
        objects[k]["correctedOdomPose"].pose = correctedWorldOdom
        objects[k]["correctedOdomPose"].covariance = msg.pose.covariance
        



    
# Utility functions, since tf3d doesn't offer ways to convert between quaternion representations :(
def xyzw_to_wxyz(q):
    x, y, z, w = q
    return (w,x,y,z)

def wxzy_to_xyzw(q):
    w, x, y, z = q
    return (x,y,z,w)

def calculateAvgPos(self, msg):
    for item in dictionary:
        #get position of current item in view
        msg_pose.position.x = item.pos.x
        msg_pose.position.y = item.pos.y
        msg_pose.position.z = item.pos.z

        #get orientation of current item in view
        msg_pose.orientation.w = item.ori.w
        msg_pose.orientation.x = item.ori.x
        msg_pose.orientation.y = item.ori.y
        msg_pose.orientation.z = item.ori.z

        #update position of self based on position of current item
        self.pos[0], self.covariance[0] = self.update_value(self.pos[0], msg_pose.position.x, self.covariance[0], object_covaraince[0])
        self.pos[1], self.covariance[1] = self.update_value(self.pos[1], msg_pose.position.y, self.covariance[1], object_covaraince[1])
        self.pos[2], self.covariance[2] = self.update_value(self.pos[2], msg_pose.position.z, self.covariance[2], object_covaraince[2])
        

        #Define the roll pitch and yaw
        msg_roll, msg_pitch, msg_yaw = euler.quat2euler([msg_pose.orientation.w, msg_pose.orientation.x, msg_pose.orientation.y, msg_pose.orientation.z], 'sxyz')
        
        #update roll pitch and yaw of self based on position of current item
        msg_roll = self.constrain_angle(self.roll, msg_roll)
        self.roll, self.covariance[3] = self.update_value(self.roll, msg_roll, self.covariance[3], object_covaraince[3])

        msg_pitch = self.constrain_angle(self.pitch, msg_pitch)
        self.pitch, self.covariance[4] = self.update_value(self.pitch, msg_pitch, self.covariance[4], object_covaraince[4])

        msg_yaw = self.constrain_angle(self.yaw, msg_yaw)
        self.yaw, self.covariance[5] = self.update_value(self.yaw, msg_yaw, self.covariance[5], object_covaraince[5])
        

def main(args=None):
    rclpy.init(args=args)
    node = dopeConverter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()   