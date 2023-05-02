#!/usr/bin/env python
import numpy as np
import rospy
import cv2

from cv_bridge   import CvBridge

from ros_sam.srv import Segmentation         as SegmentationSrv, \
                        SegmentationRequest  as SegmentationRequestMsg, \
                        SegmentationResponse as SegmentationResponseMsg

from ros_sam import SAM


if __name__ == '__main__':
    rospy.init_node('ros_sam')

    model = rospy.get_param('~model', 'vit_h')
    cuda  = rospy.get_param('~cuda', 'cuda')

    bridge = CvBridge()

    print('Starting SAM...')

    sam = SAM(model, cuda)

    def srv_segmentation(req : SegmentationRequestMsg):
        img    = cv2.cvtColor(bridge.imgmsg_to_cv2(req.image), cv2.COLOR_BGR2RGB)
        points = np.vstack([(p.x, p.y) for p in req.query_points])
        labels = np.asarray(req.query_labels)
        masks, scores, logits = sam.segment(img, points, labels, req.multimask)
        
        res = SegmentationResponseMsg()
        res.masks  = [bridge.cv2_to_imgmsg(m.astype(np.uint8)) for m in masks]
        res.scores = scores.tolist()
        if req.logits:
            res.logits = [bridge.cv2_to_imgmsg(l) for l in logits]
        return res

    srv = rospy.Service('~segment', SegmentationSrv, srv_segmentation)

    print('SAM is ready')

    while not rospy.is_shutdown():
        rospy.sleep(0.2)
    