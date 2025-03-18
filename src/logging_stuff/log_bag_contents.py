import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import struct

def pointcloud_callback(msg):
    # Decode the PointCloud2 message into (x, y, z, rgb)
    points = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)

    # Print out the first 10 points and decode the RGB values
    for i, point in enumerate(points):
        if i >= 10:  # Limit to first 10 points for simplicity
            break
        x, y, z, rgb_float = point
        
        # Convert the floating point RGB into an integer (by using struct.unpack)
        rgb_int = struct.unpack('I', struct.pack('f', rgb_float))[0]
        
        # Extract individual color channels (red, green, blue)
        r = (rgb_int >> 16) & 0xFF
        g = (rgb_int >> 8) & 0xFF
        b = rgb_int & 0xFF

        print(f"Point {i}: x={x}, y={y}, z={z}, rgb={rgb_int} (R: {r}, G: {g}, B: {b})")

rospy.init_node('pointcloud_reader')
rospy.Subscriber("/semantic_pointcloud", PointCloud2, pointcloud_callback)
rospy.spin()



"""
import rosbag
import rospy

def log_message_types_in_rosbag(bagfile_path):
    # Open the rosbag file in read mode
    with rosbag.Bag(bagfile_path, 'r') as bag:
        # Set to track unique message types
        message_types = set()

        # Iterate through all messages in the bag
        for topic, msg, t in bag.read_messages():
            # Get the message type from the msg object
            msg_type = type(msg).__name__
            message_types.add(msg_type)
        
        # Log the unique message types found in the bag
        rospy.loginfo("Message types found in the bag:")
        for msg_type in message_types:
            rospy.loginfo("- %s", msg_type)

if __name__ == "__main__":
    rospy.init_node("log_rosbag_message_types")
    
    # Path to the rosbag file
    bagfile_path = '/home/avaniab/catkin_ws/src/Kimera-Semantics/kimera_semantics_ros/rosbags/kimera_semantics_demo.bag'
    
    # Call the function to log message types
    log_message_types_in_rosbag(bagfile_path)
"""