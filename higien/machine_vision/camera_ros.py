# SYSTEM
import time
import sys

#
import numpy as np
import cv2 as cv
import pyrealsense2.pyrealsense2 as rs

# ROS
import rclpy
import tf2_ros
import tf2_geometry_msgs
import message_filters

from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PointStamped

from cv_bridge import CvBridge

# LOCAL
from .yolov5 import Yolov5
from .utilities import save_csv, line_count


class Camera(Node):

  def __init__(self):
    super().__init__("human_detection_node")  # type: ignore
    # ros2 topics
    self.aligned_depth_topic = "/camera/aligned_depth_to_color/image_raw"
    self.bgr_topic = "/camera/color/image_raw"
    self.aligned_info_topic = "/camera/aligned_depth_to_color/camera_info"

    # yolov5 model
    self.model = Yolov5()

    self.enabled = False
    self.enable_service = self.create_service(Trigger,
                                              'higien/detection/enable',
                                              self.enable)
    self.disable_service = self.create_service(Trigger,
                                               'higien/detection/disable',
                                               self.disable)

    self.get_detections_points = self.create_service(Trigger,
                                                     'higien/get_detections',
                                                     self.getDetections)
    self.humans_points = {'points': []}
    self.humans_points_marker = Marker()

    self.cycle_time = 3600.0  # seconds
    self.cycle_time_start_time = 0.0

    # Setting ros2 subscribers and callback to receive multiple topic streams
    self.bridge = CvBridge()
    # self.depth_sub = message_filters.Subscriber(self, Image, self.aligned_depth_topic)
    # self.image_sub = message_filters.Subscriber(self, Image, self.bgr_topic)

    self.depth_sub = message_filters.Subscriber(self, Image,
                                                self.aligned_depth_topic)
    self.image_sub = message_filters.Subscriber(self, Image, self.bgr_topic)

    self.info_sub = message_filters.Subscriber(self, CameraInfo,
                                               self.aligned_info_topic)

    self.subs = [self.depth_sub, self.image_sub, self.info_sub]
    self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
        self.subs, queue_size=10, slop=0.1)
    self.time_synchronizer.registerCallback(self.callback)

    self.detection_pub = self.create_publisher(
        Image, "higien/human_detection/image_raw", 10)

    self.human_point_timer = self.create_timer(5, self.publish_points)
    self.human_point_pub = self.create_publisher(
        Marker, "higien/human_detection/points", 1)

    # ROS2 frame transformation

    self.buffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.buffer, self)

    self.camera_file = "/ros2_workspaces/higien_ws/src/higien/camera_frame.csv"
    self.map_file = "/ros2_workspaces/higien_ws/src/higien/map_frame.csv"

    self.initial_point = PointStamped()
    self.transformed_point = 0
    self.line_number = line_count(self.camera_file)

    self.get_logger().info(
        'Starting {}. Please enable detection via service'.format(
            self.get_name()))

  def enable(self, request, response):
    self.get_logger().info('Incoming request to enable human_detection')
    # Prepare response
    response.success = True
    if self.enabled:
      response.message = "Higien/human_detection already enabled"
      return response

    # Suscribe to topics
    self.depth_sub = message_filters.Subscriber(self, Image,
                                                self.aligned_depth_topic)
    self.image_sub = message_filters.Subscriber(self, Image, self.bgr_topic)
    self.info_sub = message_filters.Subscriber(self, CameraInfo,
                                               self.aligned_info_topic)

    self.subs = [self.depth_sub, self.image_sub, self.info_sub]
    self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
        self.subs, queue_size=1, slop=0.1)
    self.time_synchronizer.registerCallback(self.callback)

    self.enabled = True
    response.message = "Higien/human_detection enabled"
    self.get_logger().info('Higien/human_detection enabled')

    return response

  def disable(self, request, response):
    self.get_logger().info('Incoming request to disable human_detection')
    # Prepare response
    response.success = True
    if not self.enabled:
      response.message = "Higien/human_detection already disabled"
      return response

    self.enabled = False
    # Unsuscribe from topics
    for sub in self.subs:
      self.destroy_subscription(sub.sub)

    response.message = "Higien/human_detection disabled"
    self.get_logger().info('Higien/human_detection disabled')

    return response

  def getDetections(self, request, response):
    # Save all points to file with appened date
    self.get_logger().info('Incoming request to get human_detection points')
    # Prepare response
    response.success = True
    # Save points to file
    # Get current time from ROS2 formatted as string
    current_time = self.get_clock().now().to_msg()
    current_time = str(current_time)
    current_time = current_time.replace(" ", "_")
    saveFile = self.camera_file.replace(".csv", "_{}.csv".format(current_time))
    save_csv(points, "", saveFile)

    # Clear list only if Durateion of cycle has passed  (1 hour)  (3600
    # seconds)
    if (time.time() - self.cycle_time_start_time) > self.cycle_time:
      self.humans_points['points'] = []
      self.cycle_time_start_time = time.time()

    response.message = "Higien/human_detection points saved to {}".format(
        saveFile)
    self.get_logger().info(
        'Higien/human_detection points saved to {}'.format(saveFile))

  def transform_coords(self, coords):
    # print(coords)
    try:
      x, y, _ = coords
      self.initial_point.point.x = x
      self.initial_point.point.y = y
      self.initial_point.point.z = 0.0
      trans = self.buffer.lookup_transform('map', 'camera_link',
                                           rclpy.time.Time())
      self.transformed_point = tf2_geometry_msgs.do_transform_point(
          self.initial_point, trans)
      points = [self.transformed_point.point.x, self.transformed_point.point.y]
      save_csv(points, self.map_file, self.camera_file)
      self.get_logger().info('{}: Transform: {}'.format(
          self.initial_point, self.transformed_point))

      return self.transformed_point

    except Exception as e:
      self.get_logger().info(f'Cannot transform point {e}')

  def callback(self, depth, image, info):
    # Conversion os image and depth streams to opencv format
    # self.get_logger().info('Got frame')

    image_frame = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    depth_frame = self.bridge.imgmsg_to_cv2(depth)
    # image_frame = self.bridge.compressed_imgmsg_to_cv2(
    #     image, desired_encoding="bgr8")
    # depth_frame = self.bridge.compressed_imgmsg_to_cv2(depth)

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
    if coords is not None:
      x, y, z = coords
      point = Point()
      point.x = x
      point.y = y
      point.z = z
      transformedToMap = self.transform_coords(coords)
      self.humans_points['points'].append(transformedToMap.point)

    self.model.save_csv(coords)
    # Drawing bounding boxes, class label and object distance
    image_frame = self.model.draw_object_info(image_frame, depth_frame)
    end = time.perf_counter()
    fps = 1 / np.round(end - start, 3)
    # cv.putText(image_frame, f"FPS: {fps}", (20, 70), cv.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 255), 2)

    # Publish image with bounding boxes
    image_frame = self.bridge.cv2_to_imgmsg(image_frame)
    self.detection_pub.publish(image_frame)

  def publish_points(self):
    # Publish human points
    marker = Marker()
    marker.header.stamp = self.get_clock().now().to_msg()
    marker.header.frame_id = "map"
    marker.frame_locked = True
    marker.ns = "human_points"
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.color.a = 1.0
    marker.color.g = 1.0

    marker.points = self.humans_points['points']
    self.humans_points_marker = marker
    self.human_point_pub.publish(self.humans_points_marker)
    # self.get_logger().info('Publishing: detection')


def main(args=None):
  # Initiallizing camera node
  rclpy.init(args=args)
  camera = Camera()
  rclpy.spin(camera)
  camera.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
