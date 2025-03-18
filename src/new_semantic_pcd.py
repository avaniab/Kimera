#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from sensor_msgs import point_cloud2
from threading import Lock

# Global dictionary to store the RGB values corresponding to each object 
prediction_to_rgb = {
    '010_potted_meat_can': [255, 0, 0],  # Red
    '025_mug': [0, 255, 0],             # Green
    '005_tomato_soup_can': [0, 0, 255],  # Blue
    '006_mustard_bottle': [255, 255, 0], # Yellow
    '024_bowl': [0, 255, 255],           # Cyan
    '021_bleach_cleanser': [255, 0, 255], # Magenta
    '019_pitcher_base': [50, 205, 50],   # Lime Green
    '029_plate': [255, 127, 80],         # Coral
    '009_gelatin_box_new': [255, 165, 0]  # Orange
}
object_id_to_prediction = {}

# Publisher for the updated point cloud
pointcloud_pub = None

# Global dictionary to store points grouped by object_id
points_by_object_id = {}

# Lock to ensure thread safety for shared variables
points_lock = Lock()

def prediction_callback(msg):
    """Callback to process prediction and update object-to-prediction mapping."""
    rospy.loginfo_once(f"msg data prediction {msg}")
    if msg.data:
        prediction, object_id = msg.data.split(":")
        prediction = str(prediction)
        object_id = str(object_id)
        object_id_to_prediction[object_id] = prediction
        rospy.loginfo_once(f"Updated prediction for object {prediction}:{object_id}")

def pointcloud_callback(msg):
    """Callback to process the PointCloud2 message and update with RGB values."""
    global points_by_object_id
    try:
        # Read point cloud data (x, y, z, object_id)
        pcd_data = pc2.read_points(msg, field_names=("x", "y", "z", "object_id"), skip_nans=True)

        if not object_id_to_prediction:
            rospy.logwarn("No object predictions available.")
            return

        # Group points by object_id
        with points_lock:
            for point in pcd_data:
                if len(point) == 4:
                    x, y, z, object_id = point
                    object_id = str(object_id)
                else:
                    rospy.logwarn(f"Point missing object_id at position ({point[0]}, {point[1]}, {point[2]}), skipping.")
                    continue

                # Add point to corresponding object_id group
                if object_id not in points_by_object_id:
                    points_by_object_id[object_id] = []
                    rospy.loginfo(f"Added object id {object_id}")

                points_by_object_id[object_id].append([x, y, z])
        rospy.loginfo(f"Grouped points by object_id. Total unique object_ids: {len(points_by_object_id)}")

        recolor_points_by_object_id()

    except Exception as e:
        rospy.logerr(f"Error in pointcloud callback: {str(e)}")

def recolor_points_by_object_id():
    """Recolor the grouped points by looking up object ID in the prediction-to-RGB mapping."""
    global points_by_object_id, all_points
    all_points = []  # Reset the list of all points for the new aggregation

    with points_lock:
        for object_id, points in points_by_object_id.items():
            # Lookup prediction for this object_id
            prediction = object_id_to_prediction.get(object_id, None)
            rospy.logwarn(f"object_id_to_prediction dict {object_id_to_prediction}")
            if prediction:
                rgb_value = prediction_to_rgb.get(prediction, [255, 255, 255])  # Default to white if not found
                rospy.loginfo(f"Recoloring {len(points)} points for object ID {object_id} with color {rgb_value}.")
            else:
                rgb_value = [255, 255, 255]  # Default to white if no prediction
                rospy.logwarn(f"defaulted to white for object id {object_id}")

            # Pack the RGB values as an integer (0xRRGGBB)
            rgb_value_packed = (rgb_value[0] << 16) | (rgb_value[1] << 8) | rgb_value[2]

            # Add the recolored points to the list
            for point in points:
                x, y, z = point
                all_points.append([x, y, z, rgb_value_packed])

        rospy.loginfo(f"Recolored and aggregated {len(all_points)} points.")

    # Publish the aggregated point cloud with updated colors
    publish_aggregated_pointcloud()

def publish_aggregated_pointcloud():
    """Publish accumulated point cloud data."""
    global all_points

    try:
        rospy.loginfo("Publishing aggregated PointCloud2.")

        if all_points:
            rospy.loginfo(f"Publishing {len(all_points)} accumulated points.")

            # Create a new PointCloud2 message with the accumulated RGB values
            header = rospy.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'world'  # You may need to update this frame ID based on your setup

            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.UINT32, 1),  # RGB is now packed as uint32
            ]

            # Create a PointCloud2 message using the accumulated points
            updated_pc = point_cloud2.create_cloud(header, fields, all_points)

            # Republish the updated PointCloud2 message
            pointcloud_pub.publish(updated_pc)
            rospy.loginfo("Published aggregated PointCloud2 with new RGB values.")

        else:
            rospy.logwarn("No points accumulated yet.")

    except Exception as e:
        rospy.logerr(f"Error in publish_aggregated_pointcloud: {str(e)}")

def main():
    rospy.init_node('new_semantic_pcd')

    # Create the publisher for the updated PointCloud2
    global pointcloud_pub
    pointcloud_pub = rospy.Publisher('/updated_semantic_pointcloud', PointCloud2, queue_size=10)

    # Subscribe to the obj_rgb topic to get color data
    rospy.Subscriber('/prediction', String, prediction_callback)

    # Subscribe to the individual_obj_pcd_w_obj_id topic to get the point cloud data
    rospy.Subscriber('/individual_obj_pcd_w_obj_id', PointCloud2, pointcloud_callback)

    # Keep the node running so callbacks are executed
    rospy.spin()

if __name__ == '__main__':
    main()
