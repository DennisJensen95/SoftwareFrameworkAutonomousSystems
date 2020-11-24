#!/usr/bin/env python
import rospy
from tf.transformations import quaternion_from_matrix
import tf
import math
from geometry_msgs.msg import Pose
import signal
import threading
import sys
import traceback
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class FrameUtilities():

    def __init__(self, QrCodeUtil):

        # Logging
        self.log_tag = "[FrameUtilities]: "

        # Signal handler
        signal.signal(signal.SIGINT, self.signal_handler)

        self.qr_code_util = QrCodeUtil

        # Initializing tf broadcaster
        self.tf_listener = tf.TransformListener()
        self.kill_broadcast_tf = False
        self.pose_diff_hidden_frame_to_odom = None
        self.hidden_frame_tf_thread = threading.Thread(
            target=self.broadcast_transform_hidden_to_odom)
        self.hidden_frame_tf_thread.start()

        # Except hook
        sys.excepthook = self.kill_thread_except_hook

    def log(self, msg):
        """[summary]
        Generic log message
        Args:
            msg ([str]): [Log message]
        """
        print(self.log_tag + str(msg))

    def signal_handler(self, sig, frame):
        self.log("You pressed Ctrl+C!")
        self.kill_broadcast_tf = True
        sys.exit(0)

    def kill_thread_except_hook(self, etype, value, tb):
        self.kill_broadcast_tf = True
        traceback.print_exception(etype, value, tb)
        sys.exit(0)

    def teardown(self):
        """[summary]
        Teardown threading
        """
        self.hidden_frame_tf_thread.join()

    def aquire_frame_transformations(self):
        """[summary]
        If transform buffer is not filled, try waiting and getting it again.
        Returns:
            [Bool]: [State]
        """
        self.log("Sleeping to get transform frames")
        rospy.sleep(1)
        return True

    def transform_pose_in_frames(self, pose, target_frame, current_frame):
        """[summary]
        Transform a Pose from one frame to another
        Args:
            pose ([PoseStamp]): [PoseStamp that needs transformation]
            target_frame ([str]): [String of target frame]
            current_frame ([str]): [String of current frame]

        Returns:
            [PoseStamp]: [PoseStamp of the pose in target frame]
        """
        now = rospy.Time.now()
        self.tf_listener.waitForTransform(
            target_frame, current_frame, now, rospy.Duration(4.0))
        pose_odo_frame = self.tf_listener.transformPose(target_frame, pose)

        return pose_odo_frame

    def broadcast_transform_hidden_to_odom(self):
        """[summary]
        Broadcasting transform from hidden to odometry frame when found
        """
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():

            if (self.qr_code_util.get_number_of_qr_codes() >= 2):
                if self.qr_code_util.is_qr_codes_data_updated():
                    self.create_transform_from_odom_to_hidden_frame()
                    self.qr_code_util.set_updated_qr_codes(False)
                rotation = self.pose_diff_hidden_frame_to_odom.orientation
                rot_tuple = (rotation.x, rotation.y, rotation.z, rotation.w)
                translation = self.pose_diff_hidden_frame_to_odom.position
                trans_tuple = (translation.x, translation.y, translation.z)
                try:

                    br.sendTransform(trans_tuple, rot_tuple,
                                     rospy.Time.now(), "hidden_frame", "odom")
                    rate.sleep()
                except rospy.exceptions.ROSInterruptException:
                    pass

            if self.kill_broadcast_tf:
                break

    def create_transform_from_odom_to_hidden_frame(self):
        """[summary]
        Create a transform for transforming from odometry frame to hidden frame
        Args:
            qr_code_position_odo ([Pose]): [QR code position in odemetry coordinates]
        """

        A = []
        B = []
        qr_codes_messages = self.qr_code_util.return_qr_code_data()
        for key in qr_codes_messages.keys():
            data = qr_codes_messages[key]
            A.append([data["pos"][0], data["pos"][1], 0])
            B.append([data["odom_pos"].x, data["odom_pos"].y, 0])

        A = np.transpose(np.array(A))
        B = np.transpose(np.array(B))
        (R, t) = self.rigid_transform_3D(A, B)
        R = np.array(R)
        R = np.append(R, np.array([[0, 0, 0]]), axis=0)
        R = np.append(R, np.array([[0], [0], [0], [1]]), axis=1)
        t = t[:, 0]

        quaternion_change = quaternion_from_matrix(R)
        self.pose_diff_hidden_frame_to_odom = Pose()
        self.pose_diff_hidden_frame_to_odom.position.x = t[0]
        self.pose_diff_hidden_frame_to_odom.position.y = t[1]
        self.pose_diff_hidden_frame_to_odom.position.z = t[2]
        self.pose_diff_hidden_frame_to_odom.orientation.x = quaternion_change[0]
        self.pose_diff_hidden_frame_to_odom.orientation.y = quaternion_change[1]
        self.pose_diff_hidden_frame_to_odom.orientation.z = quaternion_change[2]
        self.pose_diff_hidden_frame_to_odom.orientation.w = quaternion_change[3]

    def switch_camera_optical_link_to_base_footprint(self, pose_stamp, robot_imu_pose):
        pose_stamp.header.frame_id = "/base_footprint"
        orig_x = pose_stamp.pose.position.x
        orig_y = pose_stamp.pose.position.y
        orig_z = pose_stamp.pose.position.z
        pose_stamp.pose.position.x = orig_z
        pose_stamp.pose.position.y = -orig_x
        pose_stamp.pose.position.z = 0
        (roll, pitch, yaw) = self.convert_orientation_to_euler(
            pose_stamp.pose.orientation)
        # Turning camera optical frame to base_footprint frame
        roll = roll - math.pi
        yaw = yaw + math.pi

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        # pose_stamp.pose.orientation.x = quaternion[0]
        # pose_stamp.pose.orientation.y = quaternion[1]
        # pose_stamp.pose.orientation.z = quaternion[2]
        # pose_stamp.pose.orientation.w = quaternion[3]
        pose_stamp.pose.orientation = robot_imu_pose.orientation
        return pose_stamp

    def convert_orientation_to_euler(self, orientation):
        quaternion = (orientation.x, orientation.y,
                      orientation.z, orientation.w)
        return euler_from_quaternion(quaternion)

    def transform_qr_code_to_desired_pos(self, qr_code_pos, robot_imu_pose, dist_from, tries=2):
        """[summary]
        Transform QR code position in /CAMERA_OPTICAL_LINK frame to /ODOM frame

        Args:
            qr_code_pos ([PoseStamp]): [QR code position in /map frame]
            dist_from ([float]): [Distance to get coordinate from object]
            tries (int, optional): [Tries to transform coordinates]. Defaults to 2.

        Returns:
            [bool]: [Success]
        """
        self.log("Distance to QR code: " + str(qr_code_pos.pose.position.z))

        qr_code_pos = self.switch_camera_optical_link_to_base_footprint(
            qr_code_pos, robot_imu_pose)

        num_try = 0
        while True:
            num_try += 1
            self.log("Try: " + str(num_try))
            try:
                desired_pose = self.transform_pose_in_frames(
                    qr_code_pos, '/odom', '/base_footprint')
                break
            except Exception:
                if num_try == tries:
                    return
                rospy.sleep(1)

        (_, _, yaw) = self.convert_orientation_to_euler(
            qr_code_pos.pose.orientation)

        desired_pose.pose.position.x = desired_pose.pose.position.x - \
            math.cos(yaw)*dist_from
        desired_pose.pose.position.y = desired_pose.pose.position.y - \
            math.sin(yaw)*dist_from

        self.log(desired_pose)
        return desired_pose

    def rigid_transform_3D(A, B):
        """[summary]
        Will make a transformation from one frame to another based on a number of points.
        At least two.

        Args:
            A ([Matrix]): [Matrix containing coordinates of points in one frame]
            B ([type]): [Matrix containing coordinates of points in another frame]

        Raises:
            Exception: [If not enoguh points is present in either frame]
            Exception: [If not enoguh points is present in either frame]

        Returns:
            [tuple]: [Tuple of rotation and translation from one frame to the other]
        """
        assert A.shape == B.shape

        num_rows, num_cols = A.shape
        if num_rows != 3:
            raise Exception("matrix A is not 3xN, it is %dx%d", num_rows, num_cols)

        num_rows, num_cols = B.shape
        if num_rows != 3:
            raise Exception("matrix B is not 3xN, it is %dx%d", num_rows, num_cols)

        # find mean column wise
        centroid_A = np.mean(A, axis=1)
        centroid_B = np.mean(B, axis=1)

        # ensure centroids are 3x1
        centroid_A = centroid_A.reshape(-1, 1)
        centroid_B = centroid_B.reshape(-1, 1)

        # subtract mean
        Am = A - centroid_A
        Bm = B - centroid_B

        H = np.matmul(Am, np.transpose(Bm))

        # sanity check
        # if linalg.matrix_rank(H) < 3:
        #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

        # find rotation
        U, S, Vt = np.linalg.svd(H)
        R = np.matmul(Vt.T, U.T)

        # special reflection case
        if np.linalg.det(R) < 0:
            print("det(R) < R, reflection detected!, correcting for it ...")
            Vt[2, :] *= -1
            R = np.matmul(Vt.T, U.T)

        t = np.matmul(-R, centroid_A) + centroid_B

        return R, t

