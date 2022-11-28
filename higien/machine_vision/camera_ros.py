import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
import cv2 as cv
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import time
import sys
from yolov5 import *

from std_srvs.srv import Trigger


class Camera(Node):
    def __init__(self):
        super().__init__("camera_node")  # type: ignore
        # ros2 topics
        aligned_depth_topic = "/camera/aligned_depth_to_color/image_raw"
        bgr_topic = "/camera/color/image_raw"
        aligned_info_topic = "/camera/aligned_depth_to_color/camera_info"

        # yolov5 model
        self.model = Yolov5()

        self.enabled = False
        self.enable_service = self.create_service(Trigger, 'higien/enable', self.enable)
        self.disable_service = self.create_service(Trigger, 'higien/disable', self.disable)
        
        self.humans_found = {'points': []}
        self.cycle_time = 3600 # seconds
        self.cycle_time_start_time = 0

        # Setting ros2 subscribers and callback to receive multiple topic streams
        self.bridge = CvBridge()
        self.depth_sub = message_filters.Subscriber(self, Image, aligned_depth_topic)
        self.image_sub = message_filters.Subscriber(self, Image, bgr_topic)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, aligned_info_topic)
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.image_sub, self.info_sub], queue_size=1, slop=0.1)
        self.time_synchronizer.registerCallback(self.callback)
        self.get_logger().info('Starting {}'.format(self.get_name()))

        self.detection_pub = self.create_publisher(Image, "higien/detection", 10)


    def enable(self,request, response):
        self.get_logger().info('Incoming request to enable human_detection')
        # Prepare response
        response.success = True
        if self.enabled:
            response.message = "Higien/human_detection already enabled"
            return response
        
        self.enabled = True
        response.message = "Higien/human_detection enabled"
        self.get_logger().info('Higien/human_detection enabled')

        return response

    def disable(self,request, response):
        self.get_logger().info('Incoming request to disable human_detection')
        # Prepare response
        response.success = True
        if not self.enabled:
            response.message = "Higien/human_detection already disabled"
            return response
        
        self.enabled = False
        response.message = "Higien/human_detection disabled"
        self.get_logger().info('Higien/human_detection disabled')

        return response

    def callback(self, depth, image, info):
        # Conversion os image and depth streams to opencv format
        # self.get_logger().info('Got frame')

        image_frame = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        depth_frame = self.bridge.imgmsg_to_cv2(depth)

        # Setting camera intrinsics
        start = time.perf_counter()
        intrinsics = rs.intrinsics()
        intrinsics.width = info.width
        intrinsics.height = info.height
        intrinsics.ppx = info.k[2]
        intrinsics.ppy = info.k[5]
        intrinsics.fx = info.k[0]
        intrinsics.fy = info.k[4]
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = [i for i in info.d]

        # Extracting bounding box coordinates, predicted object classes and object centers from image frame
        self.model.detect_object_info(image_frame)
        # Get xyz coordinates
        coords = self.model.get_xyz_coordinates(intrinsics, depth_frame)
        self.model.save_csv(coords)
        # Drawing bounding boxes, class label and object distance
        image_frame = self.model.draw_object_info(image_frame, depth_frame)
        end = time.perf_counter()
        fps = 1 / np.round(end - start, 3)
        # print(f"FPS: {fps}")

        # cv.putText(image_frame, f"FPS: {fps}", (20, 70), cv.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 255), 2)
        
        image_frame = self.bridge.cv2_to_imgmsg(image_frame)

        self.detection_pub.publish(image_frame)
        # self.get_logger().info('Publishing: detection')

        # cv.imshow("Image", image_frame)
        # key = cv.waitKey(1)
        # if key & 0xFF == ord('q'):
        #     sys.exit()


def main(args=None):
    # Initiallizing camera node
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
