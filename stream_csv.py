import rospy
from tf2_msgs.msg import TFMessage
# import tf2_ros 
from geometry_msgs.msg import TransformStamped
# import tf.transformations
import csv

class CsvPublisherNode:

    def __init__(self):
        self.pub = rospy.Publisher('/tf', TFMessage, queue_size=10)
        self.rate=rospy.Rate(1)
        #self.new_list = self.load_data('/home/avaniab/Euroc/output_logs/traj_gt.csv')

        #for row in self.new_list:
            #print(row)
            
        while not rospy.is_shutdown():
            self.publish_csv_data()
            rospy.sleep()

    #get csv data in script and print it
    def load_data(self, filename):
        mylist = []
        with open(filename) as numbers:
            numbers_data = csv.DictReader(numbers, delimiter=',')
            #next(numbers_data)
            #to skip the header if needed
            for row in numbers_data:
                    mylist.append(row)

            return mylist

        

    def convert_row_to_transform(self, row):
        transform = TransformStamped()

        # Convert timestamp from row to rospy.Time
        timestamp = float(row['#timestamp'])

        transform.header.stamp = rospy.Time.from_sec(timestamp)

        #if timestamp > 4294967295:  # Example upper limit check for 32-bit unsigned int
            #rospy.logwarn(f"Timestamp value {timestamp} is too large")
        
        # Set the rest of the transform fields
        transform.transform.translation.x = float(row['x'])
        transform.transform.translation.y = float(row['y'])
        transform.transform.translation.z = float(row['z'])
        transform.transform.rotation.w = float(row['qw'])
        transform.transform.rotation.x = float(row['qx'])
        transform.transform.rotation.y = float(row['qy'])
        transform.transform.rotation.z = float(row['qz'])

        return transform
    

    def publish_csv_data(self):
        #rospy.init_node('csv_publisher', anonymous=True)
        #pub = rospy.Publisher(topic_name, TFMessage, queue_size=10)
        data_list = self.load_data('/home/avaniab/Euroc/output_logs/traj_gt.csv')
        #rate = rospy.Rate(1) 

        while not rospy.is_shutdown():
            transforms = []
            for row in data_list:

                
                timestamp = float(row['#timestamp']) * 1e-9 # Convert ns to seconds
                transform_map = self.convert_row_to_transform(row)
                transform_map.header.frame_id = 'world'
                transform_map.child_frame_id = 'map'
                transform_map.header.stamp = rospy.Time.from_sec(timestamp)
                transforms.append(transform_map)

                transform = self.convert_row_to_transform(row)


                # Create transform from world to map
                transform_map = self.convert_row_to_transform(row)
                transform_map.header.frame_id = 'world'
                transform_map.child_frame_id = 'map'
                transform_map.header.stamp = rospy.Time.from_sec(timestamp)
                transforms.append(transform_map)

                # Create transform from map to odom
                transform_odom = self.convert_row_to_transform(row)
                transform_odom.header.frame_id = 'map'
                transform_odom.child_frame_id = 'odom'
                transform_odom.header.stamp = rospy.Time.from_sec(timestamp)
                transforms.append(transform_odom)

                # Create transform from odom to base_link
                transform_base_link = self.convert_row_to_transform(row)
                transform_base_link.header.frame_id = 'odom'
                transform_base_link.child_frame_id = 'base_link'
                transform_base_link.header.stamp = rospy.Time.from_sec(timestamp)
                transforms.append(transform_base_link)

                # Create transform from base_link to cam0
                transform_cam0 = self.convert_row_to_transform(row)
                transform_cam0.header.frame_id = 'base_link'
                transform_cam0.child_frame_id = 'cam0'
                transform_cam0.header.stamp = rospy.Time.from_sec(timestamp)
                transforms.append(transform_cam0)

                # Create transform from base_link to cam1
                transform_cam1 = self.convert_row_to_transform(row)
                transform_cam1.header.frame_id = 'base_link'
                transform_cam1.child_frame_id = 'cam1'
                transform_cam1.header.stamp = rospy.Time.from_sec(timestamp)
                transforms.append(transform_cam1)

        

            # Wrap transforms in a TFMessage
            rospy.logwarn("Num Transforms: " + str(len(transforms)))
            tf_message = TFMessage(transforms)
            self.pub.publish(tf_message)
            rospy.loginfo("Published transforms")

            if transforms:
                tf_message = TFMessage(transforms)
                self.pub.publish(tf_message)
                rospy.loginfo(f"Published {len(transforms)} transforms")

                  
            

if __name__ == '__main__':
    rospy.init_node('csv_publisher')
    try:
        node = CsvPublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("exiting")
