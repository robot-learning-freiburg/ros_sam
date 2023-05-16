#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import matplotlib.pyplot as plt

from cv_bridge import CvBridge
from pathlib   import Path

from geometry_msgs.msg import Point           as PointMsg
from std_msgs.msg      import Int32MultiArray as Int32MultiArrayMsg

from ros_sam.srv import Segmentation         as SegmentationSrv, \
                        SegmentationRequest  as SegmentationRequestMsg


def show_mask(mask, ax, random_color=False):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)
    
def show_points(coords, labels, ax, marker_size=375):
    pos_points = coords[labels==1]
    neg_points = coords[labels==0]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)   
    
def show_box(box, ax):
    x0, y0 = box[0], box[1]
    w, h = box[2] - box[0], box[3] - box[1]
    ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0,0,0,0), lw=2))   


if __name__ == '__main__':
    rospy.init_node('ros_sam_test')

    print('Waiting for SAM service...')
    rospy.wait_for_service('ros_sam/segment')
    print('Found SAM service')

    image  = cv2.imread(f'{Path(__file__).parent}/../data/test.jpg')
    image  = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    bridge  = CvBridge()
    img_msg = bridge.cv2_to_imgmsg(image)

    points  = np.array([[850, 880], 
                        [1540, 700], 
                        [1000, 1100],
                        [1500, 900]])
    
    msg_points  = [PointMsg(x=x, y=y, z=0) for (x, y) in points]
    labels      = np.array([1, 1, 0, 0])

    sam_service = rospy.ServiceProxy('ros_sam/segment', SegmentationSrv)

    try:
        res = sam_service(img_msg, msg_points, labels, 
                          Int32MultiArrayMsg(), True, True)
    except rospy.ServiceException as e:
        print(f'{e}')
        print('exception raised')
        exit(0)

    for i, (msg_mask, score) in enumerate(zip(res.masks, res.scores)):
        mask = bridge.imgmsg_to_cv2(msg_mask)
        plt.figure(figsize=(10,10))
        plt.imshow(image)
        show_mask(mask, plt.gca())
        show_points(points, labels, plt.gca())
        plt.title(f"Mask {i+1}, Score: {score:.3f}", fontsize=18)
        plt.axis('off')
        plt.show()
