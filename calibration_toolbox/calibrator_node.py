# Calibration collector for chessboard and ArUco Tag
# Author: Hongtao Wu, Xin Meng
# Johns Hopkins University
# National University of Singapore
# Date: July 15, 2021

import time
import numpy as np
import cv2
import os
import argparse

import rclpy
from rclpy.node import Node
import tf2_ros
from rclpy.node import Node

from calibration_toolbox.utils import *
from calibration_toolbox.aruco import ArUco


class CalibrationNode(Node):
    
    def __init__(self, args):
        super().__init__('calibration_node')
        
        self.data_path = args.data_path

        # Specify the names of the frames in TF
        self.base_frame_name = args.base_frame
        self.ee_frame_name = args.ee_frame
        self.num_poses = args.num_poses

        # Initialize TF
        self.tfBuffer = tf2_ros.Buffer()
        self.robot_pose_listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.camera_handler = ArUco(self, args)

        self.robot_poses = []
        self.marker_poses = []

        self.cam2ee = None

        self.data_dir = None

    def get_marker_2_cam(self):
        """Get the transformation of the marker in camera frame."""

        rclpy.spin_once(self, timeout_sec=0.5)

        markerIncam_pos, markerIncam_quat, aruco_img = self.camera_handler.get_pose()

        # If the difference between the previous marker pos and the current is large
        # Consider it got a new detection of tag
        if markerIncam_pos is not None:
            markerIncam_mat = make_rigid_transformation(markerIncam_pos, quat2rotm(markerIncam_quat))
        else:
            markerIncam_mat = None

        return markerIncam_mat, aruco_img

    def save_transforms_to_file_aruco(self, save_dir, calib_pt_idx, tool_transformation, marker_transformation, aruco_img):
        """Save the transformation to files."""

        robot_pose_file = os.path.join(save_dir, str(calib_pt_idx)  + '_robotpose.txt')
        marker_pose_file = os.path.join(save_dir, str(calib_pt_idx) + '_markerpose.txt')
        aruco_img_file = os.path.join(save_dir, str(calib_pt_idx) + '_img.png')
        
        # Tool pose in robot base frame
        with open(robot_pose_file, 'w') as file1:
            for l in np.reshape(tool_transformation, (16, )).tolist():
                file1.writelines(str(l) + ' ')

        # Marker pose in camera frame
        with open(marker_pose_file, 'w') as file2:
            for l in np.reshape(marker_transformation, (16, )).tolist():
                file2.writelines(str(l) + ' ')
        
        if aruco_img is not None:
            cv2.imwrite(aruco_img_file, aruco_img)

    def save_transforms_to_file_chessboard(self, save_dir, calib_pt_idx, tool_transformation, chessboard_img):
        """Save the transformation to files."""

        robot_pose_file = os.path.join(save_dir, str(calib_pt_idx)  + '_robotpose.txt')
        chessboard_img_file = os.path.join(save_dir, str(calib_pt_idx) + '_img.png')
        
        # Tool pose in robot base frame
        with open(robot_pose_file, 'w') as file1:
            for l in np.reshape(tool_transformation, (16, )).tolist():
                file1.writelines(str(l) + ' ')
        
        if chessboard_img is not None:
            cv2.imwrite(chessboard_img_file, chessboard_img)

    def run(self):
        """Collect data for calibration."""        
        self.get_logger().info("Ready to collect data...")
        
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        complete_point_num = 0
        while(complete_point_num < self.num_poses):

            input("Please move the robot to a new pose and press Enter to continue...")

            time.sleep(2.)
            # tf2_ros.TransformListenersleep(3.0)
            # Needed for updating subscriber as it is currently blocking
            for i in range(100):
                rclpy.spin_once(self, timeout_sec=0.01)
            print('finished spinning')

            # Marker Pose and Image
            marker_pose, aruco_img = self.get_marker_2_cam()

            if marker_pose is not None:
                # Robot Pose
                transform_ros = self.tfBuffer.lookup_transform(self.base_frame_name, self.ee_frame_name, rclpy.time.Time())

                pos_ros = transform_ros.transform.translation
                quat_ros = transform_ros.transform.rotation

                robot_pos = np.array([pos_ros.x, pos_ros.y, pos_ros.z])
                robot_quat = np.array([quat_ros.w, quat_ros.x, quat_ros.y, quat_ros.z])
                robot_rotm = quat2rotm(robot_quat)
                robot_pose = make_rigid_transformation(robot_pos, robot_rotm)

                self.save_transforms_to_file_aruco(self.data_path, complete_point_num, robot_pose, marker_pose, aruco_img)
                complete_point_num += 1

                print ("===============================")
            else:
                print ("Marker pose is None! The camera probably cannot see it!")
            rclpy.spin_once(self, timeout_sec=1)

        self.get_logger().info("Successfully collect data! Proceed to calibrate data.")

def main():

    parser = argparse.ArgumentParser(description='Calibration Node')
    parser.add_argument('--data_path', type=str, required=True, help='Path to save calibration data')
    parser.add_argument('--aruco_image_topic', type=str, default="/aruco/image", help='ROS topic for the aruco image')
    parser.add_argument('--aruco_pose_topic', type=str, default="/aruco/poses", help='ROS topic for the aruco pose')
    parser.add_argument('--ee_frame', type=str, default="panda_hand", help='End effector frame name')
    parser.add_argument('--base_frame', type=str, default="panda_link0", help='Base frame name')
    parser.add_argument('--num_poses', type=int, default=20, help='Number of calibration poses to use')
    
    args = parser.parse_args()

    rclpy.init()
    CC = CalibrationNode(args)
    CC.run()

if __name__ == '__main__':
    main()