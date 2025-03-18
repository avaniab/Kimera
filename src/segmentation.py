#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

segmentation_pub = None
segmented_image = None
label_color_map = {}

def watershed(img):

    #img = img[900:1300, 300:900]
    if img is None or img.size == 0:
        rospy.logerr("Error: Received empty image!")
        return None
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Thresholding 
    _, img_threshold = cv2.threshold(img_gray, 120, 255, cv2.THRESH_BINARY_INV)
   
    #  Dilation
    kernel = np.ones((3, 3), np.uint8)
    img_dilate = cv2.morphologyEx(img_threshold, cv2.MORPH_DILATE, kernel)
    
    distTrans = cv2.distanceTransform(img_dilate, cv2.DIST_L2, 5)
    
    _,distThresh = cv2.threshold(distTrans,15,255, cv2.THRESH_BINARY)

    distThresh = np.uint8(distThresh)
    _,labels = cv2.connectedComponents(distThresh)

    labels = np.int32(labels)
    labels = cv2.watershed(imgRGB, labels)


    output_image = np.zeros_like(imgRGB)

    num_labels = np.max(labels) + 1  #
    for label in range(1, num_labels):  
        if label not in label_color_map:
            
            label_color_map[label] = np.random.randint(0, 256, size=(3,), dtype=np.uint8)
            rospy.loginfo(f"label color map {label_color_map}")

        output_image[labels == label] = label_color_map[label]

    return output_image



def image_callback(msg):
    global segmented_image
    try:
        bridge = CvBridge()
        #  ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        segmented_image = watershed(cv_image)
        rospy.loginfo(f"recieved image shape {cv_image.shape}")
        rospy.loginfo(f"made segmented_image{segmented_image.shape}")

        if segmented_image is not None:
            segmentation_publisher(segmented_image)





    except CvBridgeError as e:
        rospy.logerr("CvBridgeError: {0}".format(e))

def segmentation_publisher(segmented_image):
    bridge = CvBridge()
    labels_normalized = np.uint8(segmented_image)
    


    img_msg = bridge.cv2_to_imgmsg(labels_normalized, encoding="rgb8")
    segmentation_pub.publish(img_msg)
    
    rospy.loginfo(f"segmented image has  shape: {segmented_image.shape}")
    

def listener():
    global segmentation_pub
    rospy.init_node('segmentation_node', anonymous=False)

    rospy.Subscriber("/device_0/sensor_1/Color_0/image/data", Image, image_callback)

    segmentation_pub = rospy.Publisher("/segmentation", Image, queue_size = 10)


    rospy.spin()


if __name__ == '__main__':
    listener()
