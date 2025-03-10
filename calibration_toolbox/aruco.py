# Class to interact with the ArUco tag
# Author: Hongtao Wu
# Johns Hopkins University
# Date: Nov 15, 2019

from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import Image

import rospy

from threading import Lock
import numpy as np
import time
import cv2

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError


class ArUco:
    """Class to interact with ArUco tag."""

    def __init__(self, args):
        self.pose_topic = args.aruco_pose_topic
        self.aruco_img_topic = args.aruco_image_topic
        self._aruco_img_sub = rospy .Subscriber(self.aruco_img_topic, Image, self._arucoimgCb)
        self._pose_sub = rospy.Subscriber(self.pose_topic, PoseArray, self._poseInfoCb)
        rospy.loginfo("Created subscribers for ArUco node")
        
        self.pose_x = None
        self.pose_y = None
        self.pose_z = None
        self.pose_qw = None
        self.pose_qx = None
        self.pose_qy = None
        self.pose_qz = None
        
        self.pos = None
        self.orn = None

        self.aruco_img = None

        self.mutex = Lock()
        self._bridge = CvBridge()
        rospy.loginfo("ArUco node is ready")

    def _poseInfoCb(self, msg):
        """Pose callback function."""

        if msg is None:
            rospy.loginfo("_poseInfoCb: msg is None!")
        else:
            rospy.loginfo(f"_poseInfoCb: got msg with size {len(msg.poses)}")
        with self.mutex:
            self.pose_x = msg.poses[0].position.x
            self.pose_y = msg.poses[0].position.y
            self.pose_z = msg.poses[0].position.z
            self.pose_qx = msg.poses[0].orientation.x
            self.pose_qy = msg.poses[0].orientation.y
            self.pose_qz = msg.poses[0].orientation.z
            self.pose_qw = msg.poses[0].orientation.w
            self.pos = np.array([self.pose_x, self.pose_y, self.pose_z])
            self.orn = np.array([self.pose_qw, self.pose_qx, self.pose_qy, self.pose_qz])

    def _arucoimgCb(self, msg):
        """Image callback function."""

        if msg is None:
            rospy.logwarn("_arucoimgCb: msg is None !!!!!!!!!")
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, "rgb8")
            # decode the data, this will take some time

            # rospy.loginfo('rgb color cv_image shape: ' + str(cv_image.shape) + ' depth sequence number: ' + str(msg.header.seq))
            # print('rgb color cv_image shape: ' + str(cv_image.shape))
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # rgb_img = cv2.imencode('.jpg', cv_image)[1].tobytes()
            # rgb_img = GetJpeg(np.asarray(cv_image))

            with self.mutex:
                self.aruco_img = cv_image
        except CvBridgeError as e:
            rospy.logwarn(str(e))

    def get_pose(self):
        """Get the pose of the current frame."""

        pos = None
        orn = None
        aruco_img = None

        with self.mutex:
            print('get_pose: pos: ', self.pos)
            if (self.pos is not None) and (self.orn is not None):
                pos = self.pos.copy()
                orn = self.orn.copy()
                aruco_img = self.aruco_img

        return pos, orn, aruco_img