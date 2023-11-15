#!/usr/bin/env python3
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point as PointMsg
from ros_sam_msgs.srv import Segmentation as SegmentationSrv
from ros_sam_msgs.srv import SegmentationRequest as SegmentationRequestMsg
from std_msgs.msg import Int32MultiArray as Int32MultiArrayMsg

from ros_sam import SAMClient, show_box, show_mask, show_points

if __name__ == "__main__":
    rospy.init_node("ros_sam_test")

    print("Waiting for SAM service...")
    rospy.wait_for_service("ros_sam/segment")
    print("Found SAM service")

    image = cv2.imread(f"{Path(__file__).parent}/../data/car.jpg")
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    points = np.array([[1035, 640], [1325, 610]])
    labels = np.array([0, 0])
    boxes = np.asarray([[54, 350, 1700, 1300]])

    sam = SAMClient("ros_sam")

    masks, scores = sam.segment(image, points, labels, boxes=boxes)

    for i, (mask, score) in enumerate(zip(masks, scores)):
        plt.figure(figsize=(10, 10))
        plt.imshow(image)
        show_mask(mask, plt.gca(), color=(255, 200, 40, 150))
        show_points(points, labels, plt.gca())
        for box in boxes:
            show_box(box, plt.gca())

        plt.title(f"Mask {i+1}, Score: {score:.3f}", fontsize=18)
        plt.axis("off")
        plt.show()
