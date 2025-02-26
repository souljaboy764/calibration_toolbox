# Camera Calibration Toolbox
A toolbox for hand-eye calibration for camera on a manipulator. Modified from https://github.com/hongtaowu67/calibration_toolbox
Tested for the Franka Emika Panda robot with ZED2 Cameras. This repo assumes that your camera intrinsics are calibrated for the aruco marker tracking.

## Installation

* [OpenCV](https://opencv.org/releases/)
* [ros2-aruco-pose-estimation](https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation): for aruco marker tracking.

## Extrinsic Calibration

This repository solves AXXB problem to get the extrinsic calibration problem.
It consists of 2 steps: collecting data and solving AXXB.
There are 3 options of calibration:

* ```EH```: eye-on-hand, calibrate the pose of the camera frame in the end-effector frame
* ```EBME```: eye-on-base, calibrate the pose of the marker in the end-effector frame
* ```EBCB```: eye-on-base, calibrate the pose of the camera in the base frame

### Data Collection
In the data collection part, the robot moves to different configurations and collects the transformation from the hand (end effector) to base and the corresponding transformation from camera to chessboard.

1. Generate and print an [aruco marker](https://chev.me/arucogen/).
2. Attach the target onto a flat rigid plane and fix it on 1) the end effector of the robot for ```EBCB``` 2) table for ```EH``` 3) and hand for ```EBME```.
3. Start your robot node and camera node
4. Launch the ArUco marker node, as per the instructions [here](https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation/blob/main/README.md)
5. run [calibrator_node.py](./calibration_toolbox/calibrator_node.py)
6. Move the robot to different configurations. Make sure in each configuration, the camera can see the whole target and the target is LARGE. Because if the target is too small, the estimation of pose will be inaccurate. The pose of the tag will be collected along the data collection.

### Solve AXXB to get the eye-to-hand transformation

The Park & Martin's method is used to solve AXXB problem.

1. Specify the `data_path` in [calibrator.py](./calibration_toolbox/calibrator.py) to the same one used with [calibrator_node.py](./calibration_toolbox/calibrator_node.py) during data collection.
2. Run [calibrator.py](./calibration_toolbox/calibrator.py) to solve the AXXB problem.
    The calibrated transformation will be saved in the data directory as ```pose.txt``` and ```pose.yaml```.
    Make sure to check each of the check pose in the terminal.
    If the calibration is successful, they should be very close to each other.
