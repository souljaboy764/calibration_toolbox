from calibration_toolbox.axxb import AXXBCalibrator
from calibration_toolbox.utils import *

import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser(description='Calibration Node')
parser.add_argument('--data_path', type=str, required=True, help='Path to save calibration data')
parser.add_argument('--calib_type', type=str, choices=['EBCB', 'EBME', 'EH'], required=True, help='Type of calibration')
parser.add_argument('--to_base_link', action='store_true', help='Calibrate to the camera base link')
parser.add_argument('--base_link', type=str, help='Name of the camera base link frame')
parser.add_argument('--optical_frame', type=str, help='Name of the camera optical frame')
args = parser.parse_args()

# if args. to_base_link:
#     assert args.base_link is not None, "Please provide the name of the camera base link frame"
#     assert args.optical_frame is not None, "Please provide the name of the camera optical frame"

AXXBCalib = AXXBCalibrator(args.calib_type)
AXXBCalib.load_xforms(args.data_path)

if args.to_base_link:
    # rclpy.init()
    # node = rclpy.create_node('calibration_node')
    # tf_buffer = tf2_ros.Buffer()
    # tf_listener = tf2_ros.TransformListener(tf_buffer, node)    
    # for i in range(1000):
    #     try:
    #         trans = tf_buffer.lookup_transform(args.base_link, args.optical_frame, rclpy.time.Time())
    #         translation = trans.transform.translation
    #         rotation = trans.transform.rotation
    #         T_optical_to_baselink = make_rigid_transformation(np.array([translation.x, translation.y, translation.z]), quat2rotm(np.array([rotation.w, rotation.x, rotation.y, rotation.z])))

    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         print("Failed to get transform from {} to {}".format(args.base_link, args.optical_frame))
            
    #     time.sleep(0.01)
    # For Zed2 Camera
    T_optical_to_baselink = np.eye(4)
    T_optical_to_baselink[:3, 3] = [0.060, 0.015, 0.011]
    T_optical_to_baselink[:3, :3] = R.from_quat([0.512, -0.512, 0.487, 0.487]).as_matrix()
    # T_optical_to_baselink = np.linalg.inv(T_optical_to_baselink)
else:
    T_optical_to_baselink = None
cam2ee = AXXBCalib.axxb(T_optical_to_baselink)

AXXBCalib.test()
AXXBCalib.write_pose_file()