#!/usr/bin/env python3
"""
fpv_camera_mux.py

Listen to the active robot on /active_robot (std_msgs/String),
subscribe to that robot's camera topics:

  /<robot_name>/camera/image_raw
  /<robot_name>/camera/camera_info

and forward them to fixed FPV topics:

  /fpv_camera/image_raw
  /fpv_camera/camera_info
"""

from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo


class FPVCameraMux(Node):
    def __init__(self):
        super().__init__('fpv_camera_mux')

        # Parameters so you can change suffixes if needed
        self.declare_parameter('camera_image_suffix', 'camera/image_raw')
        self.declare_parameter('camera_info_suffix', 'camera/camera_info')

        # Active robot name
        self.current_robot: Optional[str] = None

        # Subscriptions for currently active robot
        self._image_sub: Optional[rclpy.subscription.Subscription] = None
        self._info_sub: Optional[rclpy.subscription.Subscription] = None

        # Subscribe to /active_robot
        self.active_robot_sub = self.create_subscription(
            String,
            '/active_robot',
            self._on_active_robot,
            10,
        )

        # Publishers for FPV topics
        self.fpv_image_pub = self.create_publisher(
            Image, '/fpv_camera/image_raw', 10)
        self.fpv_info_pub = self.create_publisher(
            CameraInfo, '/fpv_camera/camera_info', 10)

        self.get_logger().info(
            "FPV Camera Mux started. Waiting for /active_robot..."
        )

    # ---------- Active robot handling ----------

    def _on_active_robot(self, msg: String):
        robot_name = msg.data.strip()
        if not robot_name:
            self.get_logger().warn("Received empty robot name.")
            return

        if robot_name == self.current_robot:
            return

        self.get_logger().info(f"Active robot changed to: {robot_name}")
        self._switch_camera_subscriptions(robot_name)

    def _switch_camera_subscriptions(self, robot_name: str):
        # Clean up old subs
        if self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
        if self._info_sub is not None:
            self.destroy_subscription(self._info_sub)
            self._info_sub = None

        img_suffix = self.get_parameter('camera_image_suffix')\
            .get_parameter_value().string_value.lstrip('/')
        info_suffix = self.get_parameter('camera_info_suffix')\
            .get_parameter_value().string_value.lstrip('/')

        image_topic = f'/{robot_name}/{img_suffix}'
        info_topic = f'/{robot_name}/{info_suffix}'

        self.get_logger().info(
            f"Subscribing to camera topics: {image_topic}, {info_topic}"
        )

        self._image_sub = self.create_subscription(
            Image, image_topic, self._on_image, 10
        )
        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, 10
        )

        self.current_robot = robot_name

    # ---------- Forwarders ----------

    def _on_image(self, msg: Image):
        self.fpv_image_pub.publish(msg)

    def _on_camera_info(self, msg: CameraInfo):
        self.fpv_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FPVCameraMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
