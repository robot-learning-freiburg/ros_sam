"""Implementation of the SAM client for ROS"""
import rospy
import numpy as np

from cv_bridge import CvBridge

from geometry_msgs.msg import Point           as PointMsg
from std_msgs.msg      import Int32MultiArray as Int32MultiArrayMsg

from ros_sam.srv import Segmentation         as SegmentationSrv, \
                        SegmentationRequest  as SegmentationRequestMsg


class SAMClient():
    """Client for the SAM segmentation service"""

    def __init__(self, service) -> None:
        """Initialize connection to the SAM segmentation service
        Args:
            service (string): Name of the service 'ros_sam' for segementation
        """
        self._bridge = CvBridge()
        
        rospy.wait_for_service(f'{service}/segment')

        self._srv_sam = rospy.ServiceProxy(f'{service}/segment', SegmentationSrv)

    def segment(self, img_rgb, points, labels, boxes=None):
        """Takes the input image and the input prompts with points and bounding boxes.
        Input points can be positive that represent the object to be segmented or
        negative that do not represent the object to be segmented
        
        Args:
            img_rgb (np.ndarray): RGB image that will be segmented
            points (np.array): Input prompt points for the segmentation
            labels (list): Labels corresponging to the input prompt points
                            1 for positive and 0 for negative
            boxes (np.ndarray, optional): Bounding boxes covering the objects to be
                                            segmented. Defaults to None
        Returns:
            Masks (List(np.ndarray)): Segmentation masks of the segmented object.
            Scores (List(float)): Confidence scores of the segmentation masks. 
            
            SAM outputs 3 masks, where scores gives the model's own estimation of the quality of these masks
        """
        msg_boxes = Int32MultiArrayMsg()
        if boxes is not None:
            msg_boxes.data = list(boxes.flatten().astype(int))

        res = self._srv_sam(self._bridge.cv2_to_imgmsg(img_rgb), 
                            [PointMsg(x=x, y=y, z=0) for (x, y) in points],
                            labels, 
                            msg_boxes, 
                            boxes is None, False)
        return [self._bridge.imgmsg_to_cv2(m) for m in res.masks], res.scores