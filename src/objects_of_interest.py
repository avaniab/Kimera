#!/usr/bin/env python

import rospy
import open3d as o3d
import threading
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# Lock for synchronization
lock = threading.Lock()

# Publisher for point clouds that meet the threshold
pointcloud_pub = None

# Thresholds for OBB dimensions (x, y, z)
X_THRESHOLD = 7.0
Y_THRESHOLD = 7.0
Z_THRESHOLD = 7.0

def publish_pointcloud(msg):
    """
    Publish the original point cloud message if it meets the size criteria.
    """
    try:
        # Publish the original PointCloud2 message as received in the callback
        pointcloud_pub.publish(msg)
        rospy.loginfo(f"Published point cloud with points.")
    
    except Exception as e:
        rospy.logerr(f"Error publishing point cloud: {e}")

def process_pointcloud(msg, object_id):
    """
    Process the incoming point cloud to check if it meets the size criteria and publish it.
    """
    try:
        # Read the point cloud data (x, y, z)
        pointcloud_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(pointcloud_data))

        if points.shape[0] == 0:
            rospy.logwarn(f"Object ID {object_id}: Empty point cloud. Skipping.")
            return

        # Convert to Open3D point cloud for further processing
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(points[:, :3])

        # Compute the oriented bounding box (OBB)
        obb = o3d_cloud.get_oriented_bounding_box()

        # Check if the OBB dimensions exceed the thresholds
        if (obb.extent[0] > X_THRESHOLD or 
            obb.extent[1] > Y_THRESHOLD or 
            obb.extent[2] > Z_THRESHOLD):
            rospy.logwarn(f"Object ID {object_id}: OBB dimensions exceed threshold. Not publishing point cloud.")
            return  # Do not publish the point cloud if dimensions exceed the threshold

        # If the OBB dimensions are within the threshold, publish the point cloud
        with lock:
            rospy.loginfo(f"Object ID {object_id}: OBB Dimensions (X, Y, Z): {obb.extent}")
            publish_pointcloud(msg)  # Publish the original point cloud message that meets the size criteria

    except Exception as e:
        rospy.logerr(f"Error processing point cloud for Object ID {object_id}: {e}")

def pointcloud_callback(msg):
    """
    Callback to process incoming point clouds and filter objects based on bounding box dimensions.
    """
    try:
        # Read the point cloud data (x, y, z, object_id)
        pointcloud_data = pc2.read_points(msg, field_names=("x", "y", "z", "object_id"), skip_nans=True)
        points = list(pointcloud_data)

        if len(points) == 0:
            rospy.logwarn("Received an empty point cloud. Skipping.")
            return

        object_id = points[0][3]  # Extract object ID from the first point

        # Process the point cloud and check if it should be published
        threading.Thread(target=process_pointcloud, args=(msg, object_id)).start()
    
    except Exception as e:
        rospy.logerr(f"Error in point cloud callback: {e}")

def main():
    global pointcloud_pub

    rospy.init_node('objects_of_interest')

    # Subscriber for point cloud
    rospy.Subscriber('/individual_obj_pcd_w_obj_id', PointCloud2, pointcloud_callback)
    rospy.loginfo("Subscribed to /individual_obj_pcd_w_obj_id topic.")
    
    # Publisher for point cloud
    pointcloud_pub = rospy.Publisher('/objects_of_interest', PointCloud2, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    main()
