#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import matplotlib.pyplot as plt

from cv_bridge import CvBridge
from pathlib   import Path

from geometry_msgs.msg import Point as PointMsg
from std_msgs.msg      import Int32MultiArray as Int32MultiArrayMsg
from ros_sam.srv import Segmentation         as SegmentationSrv, \
                        SegmentationRequest  as SegmentationRequestMsg


from ros_sam import SAMClient, \
                    show_mask, \
                    show_points, \
                    show_box

if __name__ == '__main__':
    rospy.init_node('ros_sam_test')

    print('Waiting for SAM service...')
    rospy.wait_for_service('ros_sam/segment')
    print('Found SAM service')

    image  = cv2.imread(f'{Path(__file__).parent}/../data/car.jpg')
    image  = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    points = np.array([[1035, 640],
                       [1325, 610]])
    labels = np.array([0, 0])
    boxes  = np.asarray([[54, 350, 1700, 1300]])

    sam = SAMClient('ros_sam')
    
    masks, scores = sam.segment(image, points, labels, boxes=boxes)
    
    for i, (mask, score) in enumerate(zip(masks, scores)):
        plt.figure(figsize=(10,10))
        plt.imshow(image)
        show_mask(mask, plt.gca(), color=(255, 200, 40, 150))
        show_points(points, labels, plt.gca())
        for box in boxes:
            show_box(box, plt.gca())

        plt.title(f"Mask {i+1}, Score: {score:.3f}", fontsize=18)
        plt.axis('off')
        plt.show()