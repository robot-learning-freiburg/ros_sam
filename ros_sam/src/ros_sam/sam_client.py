import rospy
import numpy as np

from cv_bridge import CvBridge

from geometry_msgs.msg import Point           as PointMsg
from std_msgs.msg      import Int32MultiArray as Int32MultiArrayMsg

from ros_sam_msgs.srv import Segmentation         as SegmentationSrv, \
                             SegmentationRequest  as SegmentationRequestMsg


class SAMClient():
    def __init__(self, service) -> None:
        self._bridge = CvBridge()

        rospy.wait_for_service(f'{service}/segment')

        self._srv_sam = rospy.ServiceProxy(f'{service}/segment', SegmentationSrv)

    def segment(self, img_rgb, points, labels, boxes=None):
        msg_boxes = Int32MultiArrayMsg()
        if boxes is not None:
            msg_boxes.data = list(boxes.flatten().astype(int))

        res = self._srv_sam(self._bridge.cv2_to_imgmsg(img_rgb),
                            [PointMsg(x=x, y=y, z=0) for (x, y) in points],
                            labels,
                            msg_boxes,
                            boxes is None, False)
        return [self._bridge.imgmsg_to_cv2(m) for m in res.masks], res.scores