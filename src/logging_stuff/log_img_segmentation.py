#!/usr/bin/env python
import rospy
import csv
import os
import sensor_msgs.msg
import ros_numpy

# Callback function to handle the PointCloud2 data and save it to CSV
def tsdf_callback(msg, topic_name):
    # Convert PointCloud2 to a numpy array
    pc_data = ros_numpy.numpify(msg)

    # Create a file path for the CSV file, based on the topic name
    file_path = os.path.join(csv_folder_path, f"{topic_name.replace('/', '_')}.csv")
    
    # Extract x, y, z coordinates from the PointCloud2 message
    # Adjust this part based on the specific data format and what you want to log
    points = pc_data['x'], pc_data['y'], pc_data['z']

    # Write data to CSV
    with open(file_path, 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y', 'z'])  # Write the header
        for point in zip(*points):  # Zip x, y, z coordinates
            writer.writerow(point)

def subscribe_tsdf_topics():
    # Initialize the ROS node
    rospy.init_node('tsdf_to_csv_logger', anonymous=True)

    # Path to save the CSV files
    global csv_folder_path
    csv_folder_path = rospy.get_param('~csv_folder_path', './tsdf_csvs')

    # Create the folder if it doesn't exist
    if not os.path.exists(csv_folder_path):
        os.makedirs(csv_folder_path)

    # List of TSDF topic names
    tsdf_topics = [
        '/kimera_semantics_node/tsdf_map_in',
        '/kimera_semantics_node/tsdf_map_out',
        '/kimera_semantics_node/tsdf_pointcloud',
        '/kimera_semantics_node/tsdf_slice'
    ]

    # Subscribe to each TSDF topic
    for topic in tsdf_topics:
        rospy.Subscriber(topic, sensor_msgs.msg.PointCloud2, tsdf_callback, callback_args=topic)
        rospy.loginfo(f"Subscribed to topic: {topic}")

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_tsdf_topics()
    except rospy.ROSInterruptException:
        pass
