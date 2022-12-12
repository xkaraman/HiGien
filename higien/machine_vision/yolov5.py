import torch
import numpy as np
import cv2 as cv
import time
import pyrealsense2.pyrealsense2 as rs
# from sort import *
# from sort import Sort
import csv
import os


class Yolov5():

  def __init__(self):
    # Loading pretrained model
    self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n6')

    self.classes = self.model.names
    self.camera_file = "/ros2_workspaces/higien_ws/src/higien/camera_frame.csv"
    if torch.cuda.is_available():
      self.device = 'cuda'
    else:
      self.device = 'cpu'
      print('Cuda not available!')

    # self.mot_tracker = Sort()

    # Setting detection threshold
    self.detection_threshold = 0.7

    # Intiallizing lists for containing bounding box coordinates, class labels, object centers and object distance
    self.obj_boxes = []
    self.obj_ids = []
    self.obj_classes = []
    self.obj_centers = []
    self.obj_confs = []
    self.track_bbs_ids = 0

  def class_to_label(self, id):
    return self.classes[int(id)]

  def detect_object_info(self, bgr_frame):
    self.model.to(self.device)
    results = self.model(bgr_frame)
    # self.track_bbs_ids = self.mot_tracker.update(results.pred[0].cpu().numpy())
    # Extracting object class and coordinates
    labels, coordinates = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]

    n = len(labels)
    x_shape, y_shape = bgr_frame.shape[1], bgr_frame.shape[0]

    self.obj_boxes = []
    self.obj_classes = []
    self.obj_centers = []
    self.obj_confs = []
    self.obj_ids = []

    for i in range(n):
      box = coordinates[i]
      if box[4] > self.detection_threshold:
        x1 = int(box[0] * x_shape)
        y1 = int(box[1] * y_shape)
        x2 = int(box[2] * x_shape)
        y2 = int(box[3] * y_shape)
        self.obj_boxes.append([x1, y1, x2, y2])

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        if cx >= 480:
          cx = 479
        if cy >= 640:
          cy = 639

        self.obj_centers.append([cx, cy])

        self.obj_classes.append(labels[i])

        self.obj_confs.append(box[4].item())

    # for id in range(len(self.track_bbs_ids.tolist())):
    #     coords = self.track_bbs_ids.tolist()[id]
    #     x1, y1, x2, y2 = int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3])
    #     name_idx = int(coords[4])

    #     self.obj_ids.append(name_idx)

  def get_xyz_coordinates(self, color_intrin, depth_frame):
    for box in self.obj_centers:
      cx, cy = box
      depth = depth_frame[cx, cy] / 1000
      result = rs.rs2_deproject_pixel_to_point(color_intrin, [cx, cy], depth)
      dx = result[2]
      dy = -result[0]
      dz = -result[1]
      # dx = result[0]
      # dy = result[1]
      # dz = result[2]
      # print(f"X: {dx}, Y: {dy}, Z: {dz}")

      return dx, dy, dz

  def sort_id(self):
    for obj_id in self.obj_ids:
      return obj_id

  def draw_object_info(self, bgr_frame, depth_frame):
    for box, class_id, obj_center in zip(self.obj_boxes, self.obj_classes,
                                         self.obj_centers):
      # print("Class_id", class_id)
      x1, y1, x2, y2 = box
      cx, cy = obj_center
      class_id = self.class_to_label(class_id)

      depth = depth_frame[cx, cy] / 1000

      cv.line(bgr_frame, (cx, y1), (cx, y2), (0, 255, 0), 1)
      cv.line(bgr_frame, (x1, cy), (x2, cy), (0, 255, 0), 1)

      cv.rectangle(bgr_frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
      cv.putText(bgr_frame, "{} m".format(depth), (x1 + 5, y1 + 70), 0, 1.0,
                 (125, 255, 0), 2)
      # cv.putText(bgr_frame, f"ID: {obj_id}", (x1, y1 - 10), cv.FONT_HERSHEY_PLAIN, 1.0, (0, 255, 125), 2)
      cv.putText(bgr_frame, f"Object: {class_id}", (x1 + 60, y1 - 10),
                 cv.FONT_HERSHEY_PLAIN, 1.0, (0, 255, 125), 2)

    return bgr_frame

  def save_csv(self, coords):
    headerList = ["X", "Y", "Z"]
    filename = self.camera_file
    file_exists = os.path.isfile(filename)

    if coords == None:
      dx, dy, dz = 0.0, 0.0, 0.0
      with open(self.camera_file, 'a+') as f:
        writer = csv.writer(f, delimiter=';')
        if not file_exists:
          writer.writerow(headerList)
        writer.writerow([dx, dy, dz])
    else:
      dx, dy, dz = coords
      with open(self.camera_file, 'a+') as f:
        writer = csv.writer(f, delimiter=';')
        if not file_exists:
          writer.writerow(headerList)
        writer.writerow([dx, dy, dz])
