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

# current scan id 
scan_ind = 0
# whenever a scan comes, increment by 1
# scan_counter = 0
# when scan_counter == 10, remove
num_scans = 15

ind_list = []

pc_dict = {
    "id":[],
    "points":[]
}

position = []

odom = Odometry()

def odom_callback(data):
    global odom,position
    odom = data
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    z = odom.pose.pose.position.z
    
    position.append([x,y,z])

def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    # cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    # cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg 

def open3d_pc(pcd):
    global odom,position
    x = position[-num_scans][0]
    y = position[-num_scans][1]
    z = position[-num_scans][2]
    
    mesh_frame = o3d.create_mesh_coordinate_frame(size = 5, origin=[x,y,z])

    o3d.visualization.draw_geometries([pcd, mesh_frame])

def pc_callback_loam(data):
    global scan_ind, ind_list, odom

    print("%d: Scan arrived"%(scan_ind))

    points_list = []

    for d in pc2.read_points(data, skip_nans=True):
        points_list.append([d[0], d[1], d[2], d[3]])
    
    points_id = [scan_ind] * len(points_list)

    # update ind_list
    if len(ind_list) == 0:
        ind_list.append(len(points_list))
    else:
        ind_list.append(len(points_list)+ind_list[-1])

    # add points_list as well as points_id to pc_dict
    pc_dict["id"].extend(points_id)
    pc_dict["points"].extend(points_list)
    
    # a new scan arrived, increment scan index
    scan_ind += 1

    if scan_ind >= num_scans:
        # # remove scans in FIFO format
        
        pc_dict["points"] = pc_dict["points"][ind_list[0]:-1]      
        pc_dict["id"] = pc_dict["id"][ind_list[0]:-1]

        ind_list = [a - ind_list[0] for a in ind_list]
        del ind_list[0]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(pc_dict["points"])[:, :-1])

        open3d_pc(pcd)

    # pc_pub = rospy.Publisher('/registered_pc', PointCloud2, queue_size=1)
    # msg = array_to_pointcloud2(np.array(pc_dict["points"])[:, :-1], data.header.stamp, data.header.frame_id)
    # pc_pub.publish(msg)

    
    print("")

if __name__ == '__main__':
    # initialize node
    rospy.init_node('append_velodyne_pcloud')

    rospy.Subscriber('/velodyne_cloud_registered', PointCloud2, pc_callback_loam)
    rospy.Subscriber("/aft_mapped_to_init", Odometry, odom_callback)
    rospy.spin()
# trajectoryFile.close()
        