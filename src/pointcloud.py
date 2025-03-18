#!/usr/bin/env python3

import rospy
import csv
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, String
import struct
import json
from sensor_msgs.msg import Image 
import matplotlib.pyplot as plt
import ros_numpy
from std_msgs.msg import Float32MultiArray

# CSV file to store the point cloud data
csv_file = "pointcloud_data.csv"
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["x", "y", "z", "rgb"])

# Global variables
points_by_rgb = {}
rgb_to_obj_id_mapping_int = {}  # Map RGB value to an object ID
rgb_to_obj_id_mapping_tuple = {}
current_object_id = 0  # Start assigning object IDs from 0
label_matrix = {}
image_segmentation = None #json
segmentation_img = None #raw from kimera callback 
# label = np.zeros((480, 720), dtype=int) 
image_ready = False  # Flag to check if segmentation image is ready

# Publishers
individual_obj_publisher = None
individual_obj_with_id_publisher = None
rgb_to_obj_id_publisher = None
label_matrix_publisher = None

has_published = False

def log_label_matrix_to_csv(label_matrix):
    """
    Logs the label matrix to a CSV file.
    """

    label_matrix_csv_file = "/home/avaniab/label_matrix.csv"
    with open(label_matrix_csv_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Write each row of the label matrix as a row in the CSV file
        for row in label_matrix:
            writer.writerow(row)
    
    rospy.loginfo(f"Label matrix logged to {label_matrix_csv_file}")
    rospy.loginfo(f"number of defaults: {number_of_defaults}")

def createLabelMatrixFromROSImage(segmentation_img, rgb_to_obj_id_mapping_tuple, visualize = False):
    """
    Generates label matrix based on the ROS sensor_msgs/Image and the RGB to object ID mapping
    Returns a np.array: each pixel is assigned an object ID.
    """
    global image_segmentation,number_of_defaults

    try:
        image_segmentation = ros_numpy.numpify(segmentation_img)  
    except Exception as e:
        rospy.logerr(f"Error converting ROS image: {e}")
        return None
    
    label_matrix = np.zeros((image_segmentation.shape[0], image_segmentation.shape[1]), dtype=int)
    
    rospy.loginfo(f"Generated label matrix shape: {label_matrix.shape}")
    rospy.loginfo_once(f"label matrix: {label_matrix}")
    number_of_defaults = 0
    # Iterate through pixel in the the segmentation
    for i in range(image_segmentation.shape[0]):
        for j in range(image_segmentation.shape[1]):
            rgb_color = tuple(image_segmentation[i, j])  # rgb color per pixl
            rospy.loginfo_once(f"rgb color of the pixel is: {rgb_color}")
            if rgb_color in rgb_to_obj_id_mapping_tuple:
                label_matrix[i, j] = rgb_to_obj_id_mapping_tuple[rgb_color]
            else:
                label_matrix[i, j] = 0  # Default to 0 if no mapping found
                rospy.loginfo_once(f"this is the mapping with tuple: {rgb_to_obj_id_mapping_tuple}")
                rospy.logwarn_once(f"no mapping found default to zero for rgb color {rgb_color}")
                number_of_defaults = number_of_defaults + 1
                #rospy.loginfo(f" number of defaults to zero: {number_of_defaults}")


    

    """if visualize:
        plt.figure(figsize=(10, 10))
        plt.imshow(label_matrix, cmap='jet', interpolation='nearest')
        plt.colorbar(label='Object ID')
        plt.title("Label Matrix Visualization")
        plt.show()"""
    
    return label_matrix




def pointcloud_callback(msg):
    """
    PointCloud callback. Processes the point cloud data only if segmentation image is ready.
    """
    global points_by_rgb, rgb_to_obj_id_mapping_int, rgb_to_obj_id_mapping_tuple, current_object_id, has_published, label, image_ready

    if not image_ready:
        rospy.logwarn("Segmentation image not ready, skipping point cloud processing.")
        return  # Skip point cloud processing if image is not ready
    
    rospy.loginfo(f"Received PointCloud with header: seq={msg.header.seq}, timestamp={msg.header.stamp}, frame_id={msg.header.frame_id}")
    
    # Decode the PointCloud2 message into (x, y, z, rgb)
    pointcloud_data = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    unique_object_ids = set()
    
    
    # Open the CSV file in append mode
    with open(csv_file, mode='a', newline='') as file:
        writer = csv.writer(file)

        for point in pointcloud_data:
            x, y, z, rgb_float = point  # Unpack point data
            
            # Convert the floating-point RGB into an integer 
            rgb_int = struct.unpack('I', struct.pack('f', rgb_float))[0]
            
            #  RGB components from hex
            r = (rgb_int >> 16) & 0xFF
            g = (rgb_int >> 8) & 0xFF
            b = rgb_int & 0xFF

            rgb_value_tuple = (r, g, b)
            

            rospy.loginfo_once(f"Added RGB value int {rgb_int} ( of Object ID {current_object_id}")
        
            # Write point data to CSV (x, y, z, rgb)
            writer.writerow([x, y, z, rgb_int])  

            # Group points by RGB value and assign an obj ID
            if rgb_int not in points_by_rgb:
                points_by_rgb[rgb_int] = []
                rgb_to_obj_id_mapping_int[rgb_int] = current_object_id
                rgb_to_obj_id_mapping_tuple[rgb_value_tuple] = current_object_id #for segmentaion img
                current_object_id += 1 
                
            unique_object_ids.add(rgb_to_obj_id_mapping_int[rgb_int])
            points_by_rgb[rgb_int].append((x, y, z, rgb_int))



    

    rospy.loginfo(f"Number of unique object IDs in the point cloud: {len(unique_object_ids)}")

    # After processing the data, publish the object ID mapping
    publish_rgb_to_obj_id_mapping()

    # Publish label matrix for tops processing 
    publish_label_matrix()

    # Also publish individual objects
    publish_individual_objects()

    rospy.loginfo(f"Publishing point cloud data at time {msg.header.stamp}")
    has_published = True

def segmentation_img_callback(msg):
    """
    Segmentation image callback. This is called when the segmentation image is received.
    """
    global segmentation_img, image_ready

    try:
        segmentation_img = msg
        image_ready = True  # Set the flag when image is ready
        rospy.loginfo_once("Segmentation image received.")

    
    except Exception as e:
        rospy.logwarn(f"Failed to process segmentation image: {e}")
        segmentation_img = None
        image_ready = False  

def publish_rgb_to_obj_id_mapping():
    """
      RGB-to-object ID pub 
      mapping as a JSON string 
      rgb packed as json
    """
    global rgb_to_obj_id_mapping_int

    # Convert the mapping to JSON format
    mapping_as_json = json.dumps(rgb_to_obj_id_mapping_int)

    # Publish the mapping to the '/rgb_to_object_id_mapping' topic
    rgb_to_obj_id_publisher.publish(mapping_as_json)

    rospy.loginfo("Published RGB-to-Object ID Mapping")


def publish_label_matrix():
    global segmentation_img, rgb_to_obj_id_mapping_tuple
    label_matrix = createLabelMatrixFromROSImage(segmentation_img, rgb_to_obj_id_mapping_tuple,True )
    label_matrix_uint16 = np.array(label_matrix, dtype=np.uint16)
    
    # define stuff to pub as image
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "camera_frame"

    # Create the Image message
    label_matrix_image = Image()
    label_matrix_image.header = header
    label_matrix_image.height, label_matrix_image.width = label_matrix_uint16.shape
    label_matrix_image.encoding = "mono16"  
    label_matrix_image.is_bigendian = False 
    label_matrix_image.step = label_matrix_uint16.shape[1] * 2  # 2 bytes per pixel (uint16)
    label_matrix_image.data = label_matrix_uint16.tobytes()  # Flatten  matrix and convert to bytes

    # Publish the Image message
    label_matrix_publisher.publish(label_matrix_image)

    rospy.loginfo("Published label matrix as Image message")

    
    if label_matrix_publisher is not None:

        label_matrix_publisher.publish(label_matrix_image)

        rospy.loginfo(f"Published label matrix ")
        log_label_matrix_to_csv(label_matrix)
    else:
        rospy.logwarn("Label matrix is empty, not publishing.")

    

    rospy.loginfo("Published label matrix")

def publish_individual_objects():
    global points_by_rgb, individual_obj_publisher, individual_obj_with_id_publisher

    header = Header()
    header.frame_id = "world"
    header.stamp = rospy.Time.now()  # Use current time for the header

    # Create and publish point clouds for individual objects
    for rgb, points in points_by_rgb.items():
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('object_id', 12, PointField.UINT32, 1),  # Add object_id field
        ]

        object_id = rgb_to_obj_id_mapping_int[rgb]
        grouped_points_with_ids = [(x, y, z, object_id) for (x, y, z, _) in points]

        grouped_pointcloud = pc2.create_cloud(header, fields, grouped_points_with_ids)

        # Publish the PointCloud2 message with object IDs
        individual_obj_with_id_publisher.publish(grouped_pointcloud)
        individual_obj_publisher.publish(grouped_pointcloud)
        number = len(grouped_points_with_ids)


        rospy.loginfo(f"Published {number} points for object id {object_id}")

def get_pointcloud_node():
    global individual_obj_publisher, individual_obj_with_id_publisher, rgb_to_obj_id_publisher, label_matrix_publisher

    rospy.init_node('pointcloud_node')
    rospy.Subscriber("/semantic_pointcloud", PointCloud2, pointcloud_callback)
    rospy.Subscriber('/tesse/segmentation/image_raw', Image, segmentation_img_callback)

    # Publisher for individual point clouds
    individual_obj_publisher = rospy.Publisher("/individual_obj_pcd", PointCloud2, queue_size=10)
    individual_obj_with_id_publisher = rospy.Publisher("/individual_obj_pcd_w_obj_id", PointCloud2, queue_size=10)

    # Publisher for RGB-to-object ID mapping
    rgb_to_obj_id_publisher = rospy.Publisher("/rgb_to_object_id_mapping", String, queue_size=10)
    label_matrix_publisher = rospy.Publisher("/label_matrix", Image, queue_size=10)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        get_pointcloud_node()
    except rospy.ROSInterruptException:
        pass