#!/usr/bin/env python3

import sys
sys.path.append ('/home/ksakash/misc/drone_image_stitching')
sys.path.append ('/home/ksakash/projects/APAP-Image-Stitching')

import cv2
import copy
import rospy
import numpy as np
import JPEGEncoder as en
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image as ROSImage
from laser_pkg.msg import ImagePose
from squaternion import Quaternion

from image import Image
import geometry as gm

from feature.sift import SIFTMatcher
from feature.ransac import RANSAC
from stitching.homography import Homography, final_size
from stitching.apap import Apap
from stitching.blend import uniform_blend, union_blend
from utils.mesh import get_mesh, get_vertice

class ImageStitch (object):
    def __init__ (self):
        self.pose = None
        self.result = None
        self.bridge = CvBridge ()
        self.init = False
        self.detector = cv2.xfeatures2d.SURF_create (200)
        self.matcher = cv2.FlannBasedMatcher ()
        self.height = 25
        self.count = 0
        self.dimensions = (1920, 1080)
        self.scale = 1
        self.mesh_size = 50

        # subscribe to data coming from the quadcopter
        self.pose_sub = rospy.Subscriber ("/pose", PoseStamped, self.pose_cb)
        self.image_sub = rospy.Subscriber ("/image", ROSImage, self.image_cb, queue_size=5)
        self.sub = rospy.Subscriber ("/image_pose", ImagePose, self.cb, queue_size=50)

        # to talk to the controller
        self.pose_pub = rospy.Publisher ("mavros/setpoint_position/local", PoseStamped, queue_size=10, latch=True)

        self.imageDataList = []
        self.transformation_series = []
        self.position_data = []

    def get_transformation (self, id):
        trans = self.imageDataList[id]._transformation
        for i in range (id, len (self.transformation_series)):
            trans = np.dot (trans, self.transformation_series[i])
        return trans

    def get_neighbours (self, id):
        x = self.imageDataList[id]._pose[0]
        y = self.imageDataList[id]._pose[1]

        h = self.height
        w = 0.5 * h

        dist = []
        for p in self.position_data:
            d = abs (p[0] - x) + 3 * abs (p[1] - y)
            dist.append (d)

        dist = np.array (dist)
        dist_ind = dist.argsort ()
        dist.sort ()
        totalx = np.where (np.logical_and (dist <= h, dist <= h))[0]
        totalx = dist_ind[totalx]
        totalx = totalx[:3]

        dist = []
        for p in self.position_data:
            d = abs (p[1] - y) + 3 * abs (p[0] - x)
            dist.append (d)

        dist = np.array (dist)
        dist_ind = dist.argsort ()
        dist.sort ()
        totaly = np.where (np.logical_and (dist <= w, dist <= w))[0]
        totaly = dist_ind[totaly]
        totaly = totaly[:3]

        total = np.union1d (totalx, totaly)

        if total.shape[0] == 0:
            total = np.array ([j-1])
        return total

    def pose_cb (self, data):
        w = data.pose.orientation.w
        x = data.pose.orientation.x
        y = data.pose.orientation.y
        z = data.pose.orientation.z

        pos_x = data.pose.position.x
        pos_y = data.pose.position.y
        pos_z = data.pose.position.z

        q = Quaternion (w, x, y, z)
        e = q.to_euler (degrees=True)
        roll = float (e[0])
        pitch = float (e[1])
        yaw = float (e[2])

        self.pose = [pos_x, pos_y, pos_z, yaw, roll, pitch]

    def image_cb (self, data):
        try:
            curr_img = self.bridge.imgmsg_to_cv2 (data, "bgr8")
        except CvBridgeError as e:
            print (e)
            sys.exit (-1)

        print ("pose:", self.pose)
        self.combine (curr_img, self.pose)

    # function to handle the images with low matches
    def low_matches_handler (self, image, pose):
        pass

    def get_mask_corners (self, neighbours, id):
        mask_corners = []
        for k in neighbours:
            corners = self.imageDataList[id]._corners
            trans = self.get_transformation (k)
            dst = cv2.perspectiveTransform(corners, trans)
            dst = dst.astype('int32')
            temp = [(dst[0][0][0], dst[0][0][1]),(dst[1][0][0], dst[1][0][1]),\
                    (dst[2][0][0], dst[2][0][1]),(dst[3][0][0], dst[3][0][1])]
            mask_corners += temp
        # print (mask_corners)
        mask_corners = np.array (mask_corners, dtype=np.int32)
        return mask_corners

    def get_mask (self, mask_corners, shape):
        mask_corners = cv2.convexHull (mask_corners, False)
        mask_corners = mask_corners.reshape (1,-1,2)
        mask = np.zeros (shape, dtype=np.uint8)
        ignore_mask_color = (255,)
        cv2.fillPoly (mask, mask_corners, ignore_mask_color)
        return mask

    def draw_coners (self, result):
        temp = copy.copy (result)

        for i in range (len (self.imageDataList)):
            corners = self.imageDataList[i]._corners
            trans = self.get_transformation (i)
            dst = cv2.perspectiveTransform(corners, trans)
            dst = dst.astype('int32')
            cv2.polylines(temp,[dst],True,(0,0,255))

        return temp

    def cb (self, data):
        try:
            curr_img = self.bridge.imgmsg_to_cv2 (data.image, "bgr8")
        except CvBridgeError as e:
            print (e)

        w = data.pose.pose.orientation.w
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z

        pos_x = data.pose.pose.position.x
        pos_y = data.pose.pose.position.y
        pos_z = data.pose.pose.position.z

        q = Quaternion (w, x, y, z)
        e = q.to_euler (degrees=True)
        roll = float (e[0])
        pitch = float (e[1])
        yaw = float (e[2])

        self.pose = [pos_x, pos_y, pos_z, yaw, roll, pitch]

        print ("pose:", self.pose)
        self.combine (curr_img, self.pose)

    def combine (self, data, pose):
        im = Image ()

        if self.init == False:
            self.init = True
            self.result = en.compress (data)
            im._id = self.count
            im._is_seed = True
            im._is_attached = True
            im._transformation = np.array ([[1,0,0],[0,1,0],[0,0,1]])
            im._pose = pose[:3]
            M = gm.computeUnRotMatrix (pose)
            self.result, corners = gm.warpPerspectiveWithPadding (self.result, M)
            im._corners = corners
            self.position_data.append (pose[:3])
            self.imageDataList.append (im)
            self.count += 1
            return

        image = en.compress (data)
        M = gm.computeUnRotMatrix (pose)
        image, corners = gm.warpPerspectiveWithPadding (image, M)
        im._id = self.count
        im._corners = corners
        im._pose = pose
        # self.position_data.append (im._pose[:3])
        self.imageDataList.append (im)
        total = self.get_neighbours (im._id)
        print (self.count, "neighbors:", total)
        mask_corners = self.get_mask_corners (total, im._id)
        mask_re = self.get_mask (mask_corners, self.result.shape[:2])

        gray_im = cv2.cvtColor (image, cv2.COLOR_BGR2GRAY)
        _, mask_im = cv2.threshold (gray_im, 1, 255, cv2.THRESH_BINARY)
        kp_im, desc_im = self.detector.detectAndCompute (gray_im, mask_im)

        print ("No. of keypoints in the image:", len (kp_im))

        gray_re = cv2.cvtColor (self.result, cv2.COLOR_BGR2GRAY)
        kp_re, desc_re = self.detector.detectAndCompute (gray_re, mask_re)

        print ("No. of keypoints in the result:", len (kp_re))

        matches = self.matcher.knnMatch (desc_im, desc_re, k=2)

        good = []
        for m, n in matches:
            if m.distance < 0.55 * n.distance:
                good.append(m)
        print (str (len (good)) + " Matches were Found")

        if (len (good)) <= 100:
            self.low_matches_handler (image, pose)

        matches = copy.copy (good)

        src_match = np.float32 ([kp_im[m.queryIdx].pt for m in matches])
        dst_match = np.float32 ([kp_re[m.trainIdx].pt for m in matches])

        src_pts = src_match.reshape (-1,1,2)
        dst_pts = dst_match.reshape (-1,1,2)

        ransac = RANSAC ()
        final_src, final_dst = ransac.thread (src_match, dst_match, 50)
        h_agent = Homography ()
        gh = h_agent.global_homography (final_src, final_dst)

        HomogResult = cv2.findHomography (src_pts,dst_pts,method=cv2.RANSAC)
        H = HomogResult[0]
        # H = gh

        final_w, final_h, offset_x, offset_y = final_size (image, self.result, H)
        mesh = get_mesh ((final_w, final_h), self.mesh_size + 1)
        vertices = get_vertice ((final_w, final_h), self.mesh_size, (offset_x, offset_y))

        stitcher = Apap (0, [final_w, final_h], [offset_x, offset_y], 1)
        local_homography_im = np.zeros ([final_h, final_w, 3, 3], dtype=np.float)
        local_homography_im[:,:] = H
        stitcher.local_homography2 (final_src, final_dst, vertices, local_homography_im)

        translation = np.float32 (([1,0,offset_x],[0,1,offset_y],[0,0,1]))
        fullTransformation = np.dot (translation, H)

        warpedImage = np.zeros ([final_h, final_w, 3], dtype=np.uint8)
        stitcher.local_warp2 (image, local_homography_im, warpedImage)

        warpedResImg = cv2.warpPerspective (self.result, translation, (final_w, final_h))
        self.result = np.where (warpedImage != 0, warpedImage, warpedResImg)

        # im._transformation = fullTransformation
        # im._is_attached = True
        self.imageDataList[im._id]._transformation = fullTransformation
        self.imageDataList[im._id]._is_attached = True

        self.transformation_series.append (translation)
        # self.imageDataList.append (im)
        self.position_data.append (im._pose[:3])
        cv2.imwrite ('temp/finalImage' + str (self.count) + '.jpg', self.result)
        self.count += 1

        return

if __name__ == '__main__':
    rospy.init_node ('super_stitch')
    image_stitch = ImageStitch ()
    rospy.spin ()
