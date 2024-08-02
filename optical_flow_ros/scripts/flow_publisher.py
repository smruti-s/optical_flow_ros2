import sys
import rclpy
import numpy as np
import cv2
from cv_bridge import CvBridge

import time
from sensor_msgs.msg import Image

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OpticalFlowRad
from mavros_msgs.srv import CommandBool, SetMode

from typing import Optional
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovariance,
    TwistWithCovariance,
    Pose,
    Twist,
    Point,
    Quaternion,
    Vector3,
    TransformStamped,
    Transform,
)

from optical_flow_ros.flow_opencv import OpticalFlowOpenCV

lk_params = dict(
    winSize=(15, 15),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
)

feature_params = dict(maxCorners=20, qualityLevel=0.3, minDistance=10, blockSize=7)

cv_flow = OpticalFlowOpenCV()


class OpticalFlowPublisher(Node):
    def __init__(self):
        super().__init__("optical_flow_publisher")
        self.br_ = CvBridge()
        self.publisher = self.create_publisher(OpticalFlowRad, "optical_flow", 10)
        self.create_subscription(Image, "/image_raw", self.receive_image_data, 10)
        # TODO Create subscription for getting Altitude from Range Sensor
        self.get_logger().info("OpticalFlowPublisher Node has been started.")
    
    
    def receive_image_data(self, msg: Image):
        """
        Callback function to process the image and publish optical flow data
        """
        # Convert the ROS Image message to an OpenCV image
        
        frame = self.br_.imgmsg_to_cv2(msg)
        optical_flow_msg = OpticalFlowRad()

        flow_x, flow_y, points, flow_quality, dt_us = cv_flow.calc_flow(
            img_current=frame,
            img_time_us=self.get_clock().now().nanoseconds // 1000)

        if flow_x is not None and flow_y is not None:
            optical_flow_msg.integration_time_us = int(dt_us)
            optical_flow_msg.integrated_x = flow_x
            optical_flow_msg.integrated_y = flow_y
            optical_flow_msg.integrated_xgyro = float('nan')
            optical_flow_msg.integrated_ygyro = float('nan')
            optical_flow_msg.integrated_zgyro = float('nan')
            optical_flow_msg.temperature = 0
            if flow_quality < 0: 
                optical_flow_msg.quality = 0
            else:
                optical_flow_msg.quality =  int(flow_quality)  
            optical_flow_msg.time_delta_distance_us = 10000
            optical_flow_msg.distance = 2.0 # TODO Change to get from Range sensor 
            self.publisher.publish(optical_flow_msg)
            # self.get_logger().info(f"Published optical flow message with flow_x: {flow_x}")
            # self.get_logger().info(f"Published optical flow message with flow_y: {flow_y}")
            # self.get_logger().info(f"Published optical flow message with flow_quality: {flow_quality}")
        else:
                self.get_logger().warning("Optical flow calculation failed or not ready for publishing.")


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
