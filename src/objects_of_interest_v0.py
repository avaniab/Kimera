#!/usr/bin/env python

import rospy
import open3d as o3d
import threading
import numpy as np
from voxblox_msgs.msg import Mesh
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf

# Define a lock for synchronization
lock = threading.Lock()

def calculate_distance(x1, y1, z1, x2, y2, z2):
    """Calculate Euclidean distance between two points."""
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)

def process_voxel_distances(mesh_msg, transform):
    """
    Process voxels in a voxblox mesh message, calculate distances to the base_link origin,
    and log the results along with voxel colors.
    """
    translation = transform['translation']

    for mesh_block in mesh_msg.mesh_blocks:
        try:
            voxel_index = mesh_block.index
            r, g, b = mesh_block.r, mesh_block.g, mesh_block.b

            # Compute voxel position in the world frame
            voxel_world_position = [
                voxel_index[0] * mesh_msg.block_edge_length,
                voxel_index[1] * mesh_msg.block_edge_length,
                voxel_index[2] * mesh_msg.block_edge_length
            ]

            # Calculate distance to base_link origin
            distance = calculate_distance(
                voxel_world_position[0], voxel_world_position[1], voxel_world_position[2],
                translation[0], translation[1], translation[2]
            )

            rospy.loginfo(f"Voxel at {voxel_world_position} has distance: {distance:.2f} meters")
        except AttributeError as e:
            rospy.logwarn(f"Skipping malformed voxel: {e}")

def make_bbox(msg, object_id):
    """
    Create and log the bounding box for a point cloud.
    """
    try:
        pointcloud_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(pointcloud_data))

        if points.shape[0] == 0:
            rospy.logwarn(f"Object ID {object_id}: Empty point cloud. Skipping.")
            return

        # Convert to Open3D point cloud
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(points[:, :3])

        # Compute bounding box
        bbox = o3d_cloud.get_axis_aligned_bounding_box()

        with lock:
            rospy.loginfo(f"Object ID {object_id}: Bounding Box Min: {bbox.min_bound}")
            rospy.loginfo(f"Object ID {object_id}: Bounding Box Max: {bbox.max_bound}")
    except Exception as e:
        rospy.logerr(f"Error processing bounding box for Object ID {object_id}: {e}")

def pointcloud_callback(msg):
    """
    Callback to process incoming point clouds and create bounding boxes for objects.
    """
    try:
        pointcloud_data = pc2.read_points(msg, field_names=("x", "y", "z", "object_id"), skip_nans=True)
        points = list(pointcloud_data)

        if len(points) == 0:
            rospy.logwarn("Received an empty point cloud. Skipping.")
            return

        object_id = points[0][3]  # Extract object ID from the first point
        threading.Thread(target=make_bbox, args=(msg, object_id)).start()
    except Exception as e:
        rospy.logerr(f"Error in point cloud callback: {e}")

def mesh_callback(mesh_msg):
    """
    Callback to process voxblox mesh messages.
    """
    rospy.loginfo_once("Received voxblox message")
    try:
        listener = tf.TransformListener()
        listener.waitForTransform('world', 'base_link_gt', rospy.Time(0), rospy.Duration(3.0))
        trans, rot = listener.lookupTransform('world', 'base_link_gt', rospy.Time(0))
        
        transform = {
            "translation": trans,
            "rotation": rot
        }

        process_voxel_distances(mesh_msg, transform)
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Transform error: {e}")

def main():
    rospy.init_node('objects_of_interest')

    rospy.Subscriber('/kimera_semantics_node/mesh', Mesh, mesh_callback)
    rospy.loginfo("Subscribed to /kimera_semantics_node/mesh topic.")
    
    rospy.Subscriber('/individual_obj_pcd_w_obj_id', PointCloud2, pointcloud_callback)
    rospy.loginfo("Subscribed to /individual_obj_pcd_w_obj_id topic.")
    
    rospy.spin()

if __name__ == '__main__':
    main()
