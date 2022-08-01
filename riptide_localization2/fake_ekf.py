#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu

class fakeEkf(Node):
    def __init__(self):
        super().__init__('fake_ekf')
        # self.dvlSub = self.create_subscription(TwistWithCovarianceStamped, "dvl_twist", self.dvlCb, qos_profile_sensor_data)
        # self.odomSub = self.create_subscription(Odometry, "odometry/filtered", self.odomCb, qos_profile_sensor_data)

        self.dvlPub = self.create_publisher(TwistWithCovarianceStamped, "dvl/twist", qos_profile_sensor_data)
        self.imuPub = self.create_publisher(Imu, "imu/imu/data", qos_profile_system_default)
        self.depthPub = self.create_publisher(PoseWithCovarianceStamped, "depth/pose", qos_profile_sensor_data)

        self.namespace = self.get_namespace()[1:]
        if self.namespace is None:
            self.namespace = 'tempest'

        self.timer = self.create_timer(0.01, self.tick)

    def tick(self):
        clockmsg = self.get_clock().now().to_msg()

        imuMsg = Imu()
        imuMsg.header.stamp = clockmsg
        imuMsg.header.frame_id = self.namespace + '/imu_link'
        imuMsg.orientation.x = 0.0
        imuMsg.orientation.y = 0.0
        imuMsg.orientation.z = 0.0
        imuMsg.orientation.w = 1.0
        imuMsg.orientation_covariance = [1.0, 0.0, 0.0, 
                                        0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0]
        imuMsg.angular_velocity.x = 0.0
        imuMsg.angular_velocity.y = 0.0
        imuMsg.angular_velocity.z = 0.0
        imuMsg.angular_velocity_covariance = [1.0, 0.0, 0.0, 
                                            0.0, 1.0, 0.0,
                                            0.0, 0.0, 1.0]
        imuMsg.linear_acceleration.x = 0.0
        imuMsg.linear_acceleration.y = 0.0
        imuMsg.linear_acceleration.z = 0.0
        imuMsg.linear_acceleration_covariance = [1.0, 0.0, 0.0, 
                                                0.0, 1.0, 0.0,
                                                0.0, 0.0, 1.0]

        dvlMsg = TwistWithCovarianceStamped()
        dvlMsg.header.stamp = clockmsg
        dvlMsg.header.frame_id = self.namespace + '/dvl_link'
        dvlMsg.twist.twist.linear.x = 0.0
        dvlMsg.twist.twist.linear.y = 0.0
        dvlMsg.twist.twist.linear.z = 0.0
        dvlMsg.twist.twist.angular.x = 0.0
        dvlMsg.twist.twist.angular.y = 0.0
        dvlMsg.twist.twist.angular.z = 0.0
        dvlMsg.twist.covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        
        depthMsg = PoseWithCovarianceStamped()
        depthMsg.header.stamp = clockmsg
        depthMsg.header.frame_id = self.namespace + '/pressure_link'
        depthMsg.pose.pose.position.x = 0.0
        depthMsg.pose.pose.position.y = 0.0
        depthMsg.pose.pose.position.z = 0.0
        depthMsg.pose.pose.orientation.x = 0.0
        depthMsg.pose.pose.orientation.y = 0.0
        depthMsg.pose.pose.orientation.z = 0.0
        depthMsg.pose.pose.orientation.w = 1.0
        depthMsg.pose.covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        self.imuPub.publish(imuMsg)
        self.dvlPub.publish(dvlMsg)
        self.depthPub.publish(depthMsg)

def main(args=None):
    rclpy.init(args=args)
    node = fakeEkf()
    rclpy.spin(node)


if __name__ == '__main__':
    main()   