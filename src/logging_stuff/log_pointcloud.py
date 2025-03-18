#!/usr/bin/env python

import rospy
import numpy as np
import csv
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

class PointCloudLogger:
    def __init__(self, target_time=None):
        rospy.init_node('pointcloud_logger', anonymous=True)

        # Set the target_time directly to the image timestamp in nanoseconds (53167690000 ns)
        image_timestamp_ns = 53167690000  # Image timestamp in nanoseconds

        # Directly create a rospy.Time object with the image timestamp
        self.target_time = rospy.Time.from_sec(image_timestamp_ns / 1e9)  # Convert to seconds for rospy.Time

        # Subscriber to /semantic_pointcloud topic (PointCloud2)
        self.pointcloud_sub = rospy.Subscriber("/semantic_pointcloud", PointCloud2, self.pointcloud_callback)

        self.file_counter = 0

    def pointcloud_callback(self, msg):
        try:
            # If target_time is set, only log when the timestamp matches
            if self.target_time is not None:
                incoming_time = msg.header.stamp
                if incoming_time.secs != self.target_time.secs or incoming_time.nsecs != self.target_time.nsecs:
                    rospy.logwarn(f"PointCloud timestamp {incoming_time} doesn't match the target time {self.target_time}")
                    return

            # Convert PointCloud2 message to a list of points (x, y, z, intensity, etc.)
            point_list = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)

            # Convert the point cloud to a numpy array for saving
            point_cloud_data = np.array(list(point_list))

            # Save the point cloud data as a .npy file
            npy_filename = f"pointcloud_{self.file_counter}.npy"
            np.save(npy_filename, point_cloud_data)
            rospy.loginfo(f"Logged PointCloud data to {npy_filename}")

            # Now, log the point cloud data into a CSV file
            csv_filename = f"pointcloud_{self.file_counter}.csv"
            with open(csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Write headers (optional)
                writer.writerow(["x", "y", "z", "intensity"])
                # Write data rows
                writer.writerows(point_cloud_data)
            rospy.loginfo(f"Logged PointCloud data to {csv_filename}")

            # Increment file counter for the next log
            self.file_counter += 1

        except Exception as e:
            rospy.logerr(f"Failed to process PointCloud data: {e}")

    def start(self):
        # Keep the node running
        rospy.spin()


if __name__ == '__main__':
    try:
        # Example: Log PointCloud data without a specific timestamp (None means log everything)
        target_time = None  # Set a specific time if needed

        # Create an instance of the PointCloudLogger class
        logger = PointCloudLogger(target_time=target_time)

        # Start logging
        logger.start()

    except rospy.ROSInterruptException:
        pass
