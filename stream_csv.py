import rospy
from tf2_msgs.msg import TFMessage
# import tf2_ros 
from geometry_msgs.msg import TransformStamped
import tf.transformations
import array
import csv

class CsvPublisherNode:

    def __init__(self):
        self.pub = rospy.Publisher('/tf', TFMessage, queue_size=10)
        self.new_list = self.load_data('/home/avaniab/Euroc/output_logs/traj_vio.csv')

        for row in self.new_list:
            print(row)
        self.publish_csv_data()

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
        
        timestamp = float(row['#timestamp'])*.000000001
        transform.header.stamp = rospy.Time(timestamp)
        
        if timestamp > 4294967295:  # Example upper limit check for 32-bit unsigned int
            rospy.logwarn(f"Timestamp value {timestamp} is too large")
        
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

        data_list = self.load_data('/home/avaniab/Euroc/output_logs/traj_vio.csv')
        rate = rospy.Rate(1) 

        while not rospy.is_shutdown():
            transforms = []
            for row in data_list:

                transform = self.convert_row_to_transform(row)

                transform.child_frame_id = 'map'
                transform.child_frame_id = 'base_link'
                transform.child_frame_id = 'odom'

                
                
                transforms.append(transform)


            # tf.transformations.append(transform)
            #pub.publish(transform)
            #rospy.loginfo(f"Published data: {msg.data}")
            #rate.sleep()  # Sleep to maintain the desired rate

            # Wrap transforms in a TFMessage
            tf_message = TFMessage(transforms)
            self.pub.publish(tf_message)
            
            rospy.loginfo("Published transforms")
            rate.sleep()  # Sleep to maintain the desired rate

if __name__ == '__main__':
    rospy.init_node('csv_publisher')
    try:
        node = CsvPublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("exiting")