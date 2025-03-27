"""
import rospy
from sensor_msgs.msg import Image, CameraInfo

class FrameRewriter:
    def __init__(self):
        rospy.init_node('frame_id_rewriter')

        self.sub_pub_pairs = [
            ("/camera/depth/camera_info", CameraInfo),
            ("/camera/color/image_raw", Image),
            ("/camera/depth/image_rect_raw", Image),
        ]

        self.publishers = {}
        for topic, msg_type in self.sub_pub_pairs:
            mapped_topic = topic + "_remap"
            self.publishers[topic] = rospy.Publisher(mapped_topic, msg_type, queue_size=10)
            rospy.Subscriber(topic, msg_type, self.make_callback(topic))

    def make_callback(self, topic):
        def callback(msg):
            if hasattr(msg, 'header'):
                msg.header.frame_id = "depth_cam"
            self.publishers[topic].publish(msg)
        return callback

    def run(self):
        rospy.loginfo("Rewriting frame_id to 'world' and publishing to remapped topics...")
        rospy.spin()

if __name__ == '__main__':
    node = FrameRewriter()
    node.run()
"""



"""
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageEncodingConverter:
    def __init__(self):
        rospy.init_node('image_encoding_converter')

        # Parameters (can be set via rosparam or command line)
        self.input_topic = rospy.get_param("~input_topic", "/camera/color/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/camera/color/image_rgb8")
        self.desired_encoding = rospy.get_param("~desired_encoding", "rgb8")  # or 'bgr8'

        self.bridge = CvBridge()

        rospy.loginfo(f"[converter] Subscribing to: {self.input_topic}")
        rospy.loginfo(f"[converter] Republishing as: {self.output_topic} with encoding: {self.desired_encoding}")

        self.pub = rospy.Publisher(self.output_topic, Image, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Image, self.callback)

    def callback(self, msg):
        try:
            # Convert to OpenCV format
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logwarn(f"CV bridge error: {e}")
            return

        # Convert color if needed
        try:
            if msg.encoding != self.desired_encoding:
                if self.desired_encoding == "rgb8":
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                elif self.desired_encoding == "bgr8":
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        except Exception as e:
            rospy.logerr(f"OpenCV conversion error: {e}")
            return

        try:
            new_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding=self.desired_encoding)
            new_msg.header = msg.header  # preserve frame_id and timestamp
            self.pub.publish(new_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish converted image: {e}")

if __name__ == "__main__":
    try:
        ImageEncodingConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
"""

#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from image_geometry import PinholeCameraModel

class CloudifyNode:
    def __init__(self):
        rospy.init_node('cloudify_node')

        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.depth_msg = None
        self.rgb_msg = None

        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cam_info_cb, queue_size=1)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_cb, queue_size=1)
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_cb, queue_size=1)

        self.pc_pub = rospy.Publisher("/semantic_pointcloud", PointCloud2, queue_size=1)

    def cam_info_cb(self, msg):
        self.cam_model.fromCameraInfo(msg)

    def depth_cb(self, msg):
        self.depth_msg = msg
        self.try_publish()

    def rgb_cb(self, msg):
        self.rgb_msg = msg
        self.try_publish()

    def try_publish(self):
        if self.depth_msg is None or self.rgb_msg is None:
            return

        try:
            depth_img = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding="passthrough")
            rgb_img = self.bridge.imgmsg_to_cv2(self.rgb_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", e)
            return

        height, width = depth_img.shape
        fx, fy = self.cam_model.fx(), self.cam_model.fy()
        cx, cy = self.cam_model.cx(), self.cam_model.cy()

        # Generate 3D point cloud from depth
        points = []
        for v in range(0, height, 2):  # downsample for speed
            for u in range(0, width, 2):
                d = depth_img[v, u] * 0.001  # convert mm to meters if needed
                if d == 0 or np.isnan(d):
                    continue

                x = (u - cx) * d / fx
                y = (v - cy) * d / fy
                z = d

                b, g, r = rgb_img[v, u]
                rgb = int(r) << 16 | int(g) << 8 | int(b)
                points.append([x, y, z, rgb])

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
        ]

        pc_msg = pc2.create_cloud(self.rgb_msg.header, fields, points)
        self.pc_pub.publish(pc_msg)
        rospy.loginfo_throttle(5, "Published point cloud with %d points" % len(points))

if __name__ == '__main__':
    try:
        node = CloudifyNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
