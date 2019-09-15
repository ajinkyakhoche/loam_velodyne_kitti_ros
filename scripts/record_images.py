#!/usr/bin/env python
# Author> Ajinkya Khoche

import time
import rospy
import math
import numpy as np
import tf

from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String

import matplotlib
import matplotlib.pyplot as plt
import math
import open3d as o3d

import os
import sys
import glob

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

if __name__ == '__main__':
    # initialize node
    rospy.init_node('record_images_ros')

    img_path = str(sys.argv[1])
    seq_id = str(sys.argv[2])

    f = open(os.path.join(img_path, seq_id, "times.txt"), "r")
    lines = f.readlines()
    lines = [l.split("\n")[0] for l in lines]
    f.close()
    
    img_list = glob.glob(os.path.join(img_path, seq_id, "image_2", "*.png"))
    img_list = [l.split("/")[-1] for l in img_list]
    img_list = sorted(img_list)

    left_img_pub = rospy.Publisher("/left_image", Image, queue_size=10)

    r = rospy.Rate(2) # Hz

    for ind, img_name in enumerate(img_list):
        img = cv2.imread(os.path.join(img_path, seq_id, "image_2", img_name))

        tstamp = lines[ind]
        
        img_msg = Image()
        img_msg = CvBridge().cv2_to_imgmsg(img)
        img_msg.header.stamp = rospy.Time.from_sec(float(tstamp))
        img_msg.header.frame_id = "/camera_left"

        left_img_pub.publish(img_msg)

        r.sleep()

 