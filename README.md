# ROS SAM

This package is what the name suggests: Meta's `segment-anything` wrapped in a ROS node.

## Installation

Installation is easy: 
 1. Start by cloning this package into your ROS environment. 
 2. Download the (checkpoints)[https://github.com/facebookresearch/segment-anything#model-checkpoints] for the desired SAM models to the `models` directory in this package.
 3. Install SAM by running `pip install git+https://github.com/facebookresearch/segment-anything.git`.

## Using with RQT click interface

Run the launch file:

`roslaunch ros_sam gui_test.launch`

Check the terminal and wait until the SAM model has finished loading.

There will be two windows loaded. One will have the header `rqt_image_view_seg__ImageView` and the other `rqt_image_view__ImageView`, note the lack of `_seg`. The first window is where you should click, so select the topic of the camera you want to view from the drop down. In the second window you should select `/rqt_image_segmentation/masked_image`. This is where the segmented image will be displayed.

<img src="https://github.com/ARoefer/ros_sam/blob/devel/russell/figures/interface-example.png" width=50% height=50%>

## Using ROS SAM standalone

Run the SAM ROS node using `rosrun`:

```bash
rosrun ros_sam sam_node.py
```

The node currently offers a single service `ros_sam/segment` which can be called to segment an image. Check `rossrv show ros_sam/Segmentation` for request and response specifications.

You can test SAM by starting the node and then running `rosrun ros_sam sam_test.py`.
