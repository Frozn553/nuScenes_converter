import rclpy
from rclpy.node import Node
import copy
from autoware_auto_vehicle_msgs.msg import VelocityReport
from rclpy.qos import QoSReliabilityPolicy, QoSProfile, QoSHistoryPolicy,DurabilityPolicy
from geometry_msgs.msg import TransformStamped,PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import math


class nuScenes_converter(Node):
    def __init__(self):
        super().__init__("nuScenes_converter")

        self.odometry_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odometry_listener_callback,
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        
        self.transform_subscription = self.create_subscription(
            TFMessage,
            "/tf_s",
            self.transforms_listener_callback,
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )       
        
        self.init_publisher = self.create_publisher(
            VelocityReport,
            "/vehicle/status/velocity_status",
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )

        self.initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.tf_publisher = self.create_publisher(
            TFMessage,
            "/tf_static",
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.groundtruthpose_publisher = self.create_publisher(
            PoseStamped,
            "/groundtruth_pose",
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.velocityreport_timer_callback)
        self.vehicle_odometry = None
        self.out_velocity_report = None
        self.initialpose_switch = True
        self.tf_switch = True
        

    def odometry_listener_callback(self,msg):
        self.vehicle_odometry = msg
        if self.initialpose_switch:
            self.initialpose_switch = False
            self.initialpose_publish(msg)
            self.get_logger().info("publish initialpose message")
        self.groundtruthpose_publish(msg)
        self.update_velocityreport()

    def transforms_listener_callback(self,msg):
        if self.tf_switch:
            self.tf_switch = False
            self.tf_publisher.publish(msg)
            self.get_logger().info("publish tf_ststic message")

    def velocityreport_timer_callback(self):
        if self.out_velocity_report is not None:
            self.init_publisher.publish(self.out_velocity_report)
        

    def initialpose_publish(self,msg):
        initialpose_msg = PoseWithCovarianceStamped()       
        initialpose_msg.header.frame_id = "map"
        initialpose_msg.pose = msg.pose
        self.initialpose_publisher.publish(initialpose_msg)

    def groundtruthpose_publish(self,msg):
        groundtruthpose_msg = PoseStamped()       
        groundtruthpose_msg.header = msg.header
        groundtruthpose_msg.header.frame_id = "map"
        groundtruthpose_msg.pose = msg.pose.pose
        self.groundtruthpose_publisher.publish(groundtruthpose_msg)

    def update_velocityreport(self):
        if self.vehicle_odometry is None:
            return
        velocity_report = VelocityReport()
        velocity_report.header = self.vehicle_odometry.header
        velocity_report.header.frame_id = "base_link"
        velocity_report.longitudinal_velocity = self.vehicle_odometry.twist.twist.linear.x
        self.out_velocity_report = velocity_report


def main():
    rclpy.init()
    nuscenes_converter = nuScenes_converter()
    rclpy.spin(nuscenes_converter)
    nuscenes_converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
