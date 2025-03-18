#!/usr/bin/env python3

"""
This script takes the label produced by the classifier, 
correlates that label to an RGB color, 
makes that RGB color into a single number (for ROS) and publishes it with the object ID.
"""

import rospy
from std_msgs.msg import String


def rgb_to_int(r, g, b):
    """Convert RGB values to a single 32-bit unsigned integer."""
    int_rgb = (r << 16) | (g << 8) | b
    return int_rgb

# Define RGB values for the first three object IDs
rgb_colors = [
    (255, 0, 0),   # Red for Object ID 0
    (0, 255, 0),   # Green for Object ID 1
    (0, 0, 255),   # Blue for Object ID 2
    (255, 182, 193), # pink for object id 3
    (128, 0, 128), # purple for object id 4
    (255, 255, 0), # yellow for object id 5
    (173, 216, 230), # light blue for object id 6
    (255, 105, 180) # dark pink for object id 7
]

def publish_color():
    rospy.init_node('color_publisher')
    color_pub = rospy.Publisher('/obj_new_rgb_color', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    object_id = 0  # Start with Object ID 0

    while not rospy.is_shutdown():
        # Get the RGB color for the current object_id (loop through the colors)
        if object_id < len(rgb_colors):
            r, g, b = rgb_colors[object_id]
        else:
            # If there are more objects than predefined colors, use white as a default
            r, g, b = (255, 255, 255)  # White for objects beyond the first three

        # Convert the RGB values to a single integer
        int_rgb = rgb_to_int(r, g, b)

        # Log the RGB integer value along with the object ID
        
        rospy.loginfo(f"Object ID {object_id} has RGB color: ({r}, {g}, {b}), Integer: {int_rgb}")
        
        # Publish the Object ID and RGB integer value as a string in the format "Object_ID:RGB_Value"
        color_pub.publish(f"{object_id}:{int_rgb}")

        object_id += 1
        if object_id >= len(rgb_colors):
            object_id = 0  # Reset to 0 when you loop past the last color

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_color()
    except rospy.ROSInterruptException:
        pass
