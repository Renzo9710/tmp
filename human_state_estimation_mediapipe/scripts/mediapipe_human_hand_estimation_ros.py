#!/usr/bin/env python3

# /*********************************************************************
#  *
#  *  Software License Agreement
#  *
#  *  Copyright (c) 2023,
#  *  TU Dortmund - Institute of Control Theory and Systems Engineering.
#  *  All rights reserved.
#  *
#  *  This program is free software: you can redistribute it and/or modify
#  *  it under the terms of the GNU General Public License as published by
#  *  the Free Software Foundation, either version 3 of the License, or
#  *  (at your option) any later version.
#  *
#  *  This program is distributed in the hope that it will be useful,
#  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  *  GNU General Public License for more details.
#  *
#  *  You should have received a copy of the GNU General Public License
#  *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#  *
#  *  Authors: Heiko Renz
#  *********************************************************************/

import os
import sys
import time
sys.path.append("/home/localadmin/venv/mediapipe/lib/python3.8/site-packages")
import matplotlib

matplotlib.use("TkAgg")
import cv2

import mediapipe as mp
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2

# For plot landmarks fct
from typing import List, Mapping, Optional, Tuple, Union
import numpy as np
import tf
import rospy
import rospkg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from human_state_estimation_mediapipe.msg import MsgMediapipeData
from cv_bridge import CvBridge

# Source: https://colab.research.google.com/github/googlesamples/mediapipe/blob/main/examples/hand_landmarker/python/hand_landmarker.ipynb#scrollTo=s3E6NFV-00Qt&uniqifier=1
MARGIN = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
HANDEDNESS_TEXT_COLOR = (88, 205, 54)  # vibrant green

WHITE_COLOR = (224, 224, 224)
BLACK_COLOR = (0, 0, 0)
RED_COLOR = (0, 0, 255)
GREEN_COLOR = (0, 128, 0)
BLUE_COLOR = (255, 0, 0)


class MediapipeHandEstimation:
       
    def __init__(self):
        self.show_image = True
        self.tf_br = tf.TransformBroadcaster()
        # self.right_hand_pose_pub = rospy.Publisher(
        #     "right_hand_pose/normalized", PoseArray, queue_size=10
        # )
        # self.left_hand_pose_pub = rospy.Publisher(
        #     "left_hand_pose/normalized", PoseArray, queue_size=10
        # )
        # self.right_hand_pose_pixels_pub = rospy.Publisher(
        #     "right_hand_pose/pixels", PoseArray, queue_size=10
        # )
        # self.left_hand_pose_pixels_pub = rospy.Publisher(
        #     "left_hand_pose/pixels", PoseArray, queue_size=10
        # )
        self.right_hand_pose_data_pub = rospy.Publisher(
            "right_hand_pose/data", MsgMediapipeData, queue_size=10
        )
        self.left_hand_pose_data_pub = rospy.Publisher(
            "left_hand_pose/data", MsgMediapipeData, queue_size=10
        )

        self.result = None
        self.image = None

        self.parent_frame_left = "camera"
        self.parent_frame_right = "camera"

        self.bridge = None
        self.image_sub = None
        self.landmarker = None
    
    def __del__ (self):
        self.VideoWriter.release()
        cv2.destroyAllWindows()
        print("VideoWriter released and all windows closed.")
        
    def draw_landmarks_on_image(self, rgb_image, detection_result):
        hand_landmarks_list = detection_result.hand_landmarks
        handedness_list = detection_result.handedness
        annotated_image = np.copy(rgb_image)

        # Loop through the detected hands to visualize.
        for idx in range(len(hand_landmarks_list)):
            hand_landmarks = hand_landmarks_list[idx]
            handedness = handedness_list[idx]

            # Draw the hand landmarks.
            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            hand_landmarks_proto.landmark.extend(
                [
                    landmark_pb2.NormalizedLandmark(
                        x=landmark.x, y=landmark.y, z=landmark.z
                    )
                    for landmark in hand_landmarks
                ]
            )
            solutions.drawing_utils.draw_landmarks(
                annotated_image,
                hand_landmarks_proto,
                solutions.hands.HAND_CONNECTIONS,
                solutions.drawing_styles.get_default_hand_landmarks_style(),
                solutions.drawing_styles.get_default_hand_connections_style(),
            )

            # Get the top left corner of the detected hand's bounding box.
            height, width, _ = annotated_image.shape
            x_coordinates = [landmark.x for landmark in hand_landmarks]
            y_coordinates = [landmark.y for landmark in hand_landmarks]
            text_x = int(min(x_coordinates) * width)
            text_y = int(min(y_coordinates) * height) - MARGIN

            # Draw handedness (left or right hand) on the image.
            cv2.putText(
                annotated_image,
                f"{handedness[0].category_name}",
                (text_x, text_y),
                cv2.FONT_HERSHEY_DUPLEX,
                FONT_SIZE,
                HANDEDNESS_TEXT_COLOR,
                FONT_THICKNESS,
                cv2.LINE_AA,
            )

        return annotated_image

    def est_cb(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        cv_imageRGB = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv_imageRGB)
        results = self.landmarker.detect(mp_image)
        img_cols = cv_image.shape[1]
        img_rows = cv_image.shape[0]
        
        if self.show_image:
            annotated_image = self.draw_landmarks_on_image(
                mp_image.numpy_view(), results
            )
            if annotated_image is not None:
                annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
                cv2.imshow("Mediapipe hand estimation", annotated_image)
                if self.save_video:
                    self.VideoWriter.write(annotated_image)
                key = cv2.waitKey(1)
        for i in range(len(results.handedness)):

            # Prepare the PoseArray message for each hand
            msg = PoseArray()
            msg_pixels = PoseArray()
            msg_data = MsgMediapipeData()
            msg.header.stamp = rospy.Time.now()
            msg_pixels.header.stamp = rospy.Time.now()
            msg_data.header.stamp = rospy.Time.now()

            # get wrist position to shift origin into wrist (normally in geometric center of hand)
            wrist_x = 0.0
            wrist_y = 0.0
            wrist_z = 0.0

            for j in range(len(results.hand_landmarks[i])):
                # Declare a Pose object to save x, y and z for every landmark in the hand. In this case, the quaternion is 1 in
                # w,and zeros for x, y and z
                ps = Pose()
                ps_pixels = Pose()
                # get pixel coordinates
                pixels = solutions.drawing_utils._normalized_to_pixel_coordinates(
                    results.hand_landmarks[i][j].x,
                    results.hand_landmarks[i][j].y,
                    img_cols,
                    img_rows,
                )
                if pixels is None:
                    continue
                else:
                    ps_pixels.position.x = pixels[0]
                    ps_pixels.position.y = pixels[1]
                    ps_pixels.position.z = 0

                ps.position.x = results.hand_landmarks[i][j].x - wrist_x
                ps.position.y = results.hand_landmarks[i][j].y - wrist_y
                ps.position.z = results.hand_landmarks[i][j].z - wrist_z

                ps.orientation.x = 0
                ps.orientation.y = 0
                ps.orientation.z = 0
                ps.orientation.w = 1

                # Concatenate the Pose object in the PoseArray msg

                msg.poses.append(ps)
                msg_pixels.poses.append(ps_pixels)

            # Publish the PoseArray message
            if results.handedness[i][0].display_name == "Right":
                msg.header.frame_id = self.parent_frame_right
                msg_pixels.header.frame_id = self.parent_frame_right
                msg_data.header.frame_id = self.parent_frame_right

                msg_data.pixels = msg_pixels
                msg_data.coordinates = msg

                # self.right_hand_pose_pixels_pub.publish(msg_pixels)
                # self.right_hand_pose_pub.publish(msg)
                self.right_hand_pose_data_pub.publish(msg_data)
            elif results.handedness[i][0].display_name == "Left":
                msg.header.frame_id = self.parent_frame_left
                msg_pixels.header.frame_id = self.parent_frame_left
                msg_data.header.frame_id = self.parent_frame_left

                msg_data.pixels = msg_pixels
                msg_data.coordinates = msg

                # self.left_hand_pose_pixels_pub.publish(msg_pixels)
                # self.left_hand_pose_pub.publish(msg)
                self.left_hand_pose_data_pub.publish(msg_data)
            else:
                rospy.logwarn("mediapipe_human_hand_estimation: Unknown hand detected")

        return

    def mpTask(self,save_video=True):
        BaseOptions = mp.tasks.BaseOptions
        HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("human_state_estimation_mediapipe")
        model_path = package_path + "/models/hand_landmarker.task"
        LandmarkerOptions = HandLandmarkerOptions(
            base_options=BaseOptions(
                model_asset_path=model_path, delegate=BaseOptions.Delegate.GPU
            ),
            running_mode=VisionRunningMode.IMAGE,
            num_hands=2,
        )
        self.landmarker = mp.tasks.vision.HandLandmarker.create_from_options(
            LandmarkerOptions
        )
        self.bridge = CvBridge()
        self.LandmarkerOptions = LandmarkerOptions
        self.image_sub = rospy.Subscriber(
            "mp_camera/rgb/image_rect_color", Image, self.est_cb, queue_size=1
        )

        self.save_video = save_video

        if self.save_video:
            # Initialize video writer with current date and time
            video_path = "/home/localadmin/ur10_ws/mp_hand_estimation_"
            timestr = time.strftime("%Y%m%d_%H%M%S")
            video_path += timestr
            video_path += ".mp4"
            codec = cv2.VideoWriter_fourcc(*'mp4v')
            # codec = cv2.VideoWriter_fourcc(*'h264')
            self.VideoWriter = cv2.VideoWriter(
                video_path,
                codec,
                30,
                (1280,720),
            )


if __name__ == "__main__":
    rospy.init_node("mediapipe_hand_landmark_estimation")
    estimator = MediapipeHandEstimation()

    estimator.mpTask(True) # True to save video, False else

    
    rospy.spin()
