# ROS SAM

This package is what the name suggests: Meta's `segment-anything` wrapped in a ROS node.

## Installation

Installation is easy: 
 1. Start by cloning this package into your ROS environment. 
 2. Download the (checkpoints)[https://github.com/facebookresearch/segment-anything#model-checkpoints] for the desired SAM models to the `models` directory in this package.
 3. Install SAM by running `pip install git+https://github.com/facebookresearch/segment-anything.git`.

## Using SAM

Run the SAM ROS node using `rosrun`:

```bash
rosrun ros_sam sam_node.py
```

The node currently offers a single service `ros_sam/segment` which can be called to segment an image. Check `rossrv show ros_sam/Segmentation` for request and response specifications.

You can test SAM by starting the node and then running `rosrun ros_sam sam_test.py`.
