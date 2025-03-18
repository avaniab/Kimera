#!/usr/bin/env python
import rospy
import csv
import os
import voxblox_msgs.msg

# Variable to track if data is received
data_received = False

# Callback function to handle the Layer data and save it to CSV
def tsdf_callback(msg, topic_name):
    global data_received
    rospy.loginfo(f"Received message from {topic_name}")
    data_received = True  # Mark that data has been received

    try:
        # Access the Layer data
        layer_data = msg  # 'msg' is a voxblox_msgs/Layer message
        rospy.loginfo(f"Layer data: {layer_data}")
    except Exception as e:
        rospy.logerr(f"Failed to process the message: {e}")
        return

    # Log the number of blocks in the layer (assuming 'blocks' is a field of the Layer message)
    num_blocks = len(layer_data.blocks) if hasattr(layer_data, 'blocks') else 0
    rospy.loginfo(f"Number of blocks in {topic_name}: {num_blocks}")

    # Create a file path for the CSV file, based on the topic name
    file_path = os.path.join(csv_folder_path, f"{topic_name.replace('/', '_')}.csv")

    try:
        with open(file_path, 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['block_id', 'block_data'])  # Example header, adjust based on data structure
            for block in layer_data.blocks:  # Example of iterating over blocks
                writer.writerow([block.block_id, block.block_data])  # Adjust based on block's actual attributes
            rospy.loginfo(f"Data saved to {file_path}")
    except Exception as e:
        rospy.logerr(f"Error writing to CSV: {e}")

# Timer callback to check if data has been received
def check_data_timer(event):
    global data_received
    if not data_received:
        rospy.logwarn("No data received on the topic.")
    data_received = False  # Reset flag for the next check

def subscribe_tsdf_topics():
    # Initialize the ROS node
    rospy.init_node('tsdf_to_csv_logger', anonymous=True)

    # Path to save the CSV files
    global csv_folder_path
    csv_folder_path = "/home/avaniab/catkin_ws/src/semantics_tops_integrator/src/logging_stuff/tsdf_csvs"

    # Create the folder if it doesn't exist
    if not os.path.exists(csv_folder_path):
        os.makedirs(csv_folder_path)
        rospy.loginfo(f"Created folder: {csv_folder_path}")

    # List of TSDF topic names (only subscribe to tsdf_map_out)
    tsdf_topics = [
        '/kimera_semantics_node/tsdf_map_out',  # Only subscribing to this topic now
    ]

    # Subscribe to each TSDF topic
    for topic in tsdf_topics:
        rospy.Subscriber(topic, voxblox_msgs.msg.Layer, tsdf_callback, callback_args=topic)
        rospy.loginfo(f"Subscribed to topic: {topic}")

    # Timer to check if data is received every 5 seconds
    rospy.Timer(rospy.Duration(5), check_data_timer)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_tsdf_topics()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception")
