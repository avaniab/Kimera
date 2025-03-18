#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import struct

def rgb_to_float(r, g, b):
    # Combine the RGB values into a single 32-bit unsigned integer
    int_rgb = (r << 16) | (g << 8) | b

    # Convert the 32-bit unsigned integer to a 32-bit float
    float_rgb = struct.unpack('f', struct.pack('I', int_rgb))[0]
    
    return float_rgb

# Define RGB values
#will eventually make tabel that assigns rgb value based on classifier label output
r, g, b = 255, 127, 0  # Example RGB values

# Convert the RGB values to a single float
float_rgb = rgb_to_float(r, g, b)
rospy.loginfo(f"Combined float RGB value: {float_rgb}")

def publish_color():
    rospy.init_node('color_publisher', anonymous=True)
    color_pub = rospy.Publisher('obj_new_rgb_color', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Convert the RGB values to a single float
        float_rgb = rgb_to_float(r, g, b)

        # Publish the float RGB value as a string
        color_pub.publish(str(float_rgb))

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_color()
    except rospy.ROSInterruptException:
        pass


